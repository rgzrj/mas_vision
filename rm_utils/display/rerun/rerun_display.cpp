#include "rerun_display.hpp"
#include "mas_log.hpp"
#include "rm_utils/io/runtime_path.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <unordered_map>
#include <rerun.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace rm_utils
{

struct RerunDisplay::Impl
{
    std::unique_ptr<rerun::RecordingStream> recording;
    double frame_time = NAN;
    bool shared_image = false;
    bool image_logged = false;
    bool raw_image    = false;
    bool raw_hud      = false;

    std::vector<std::uint8_t> jpeg, png;
    std::unordered_map<std::string, cv::Mat> hud_buffers;
    std::unordered_map<std::string, int> hud_heights;

    std::uint64_t image_logs = 0, hud_logs = 0;
    std::uint64_t image_bytes = 0, hud_bytes = 0;

    void log_image(const std::string &root, const cv::Mat &image);
    void log_geometry(const std::string &root, const std::vector<DisplayPoint> &points,
                      const std::vector<DisplayLine> &lines, const std::vector<DisplayText> &texts);
    void log_hud(const std::string &name, const std::string &root, const cv::Mat &image,
                 const std::vector<DisplayText> &texts);
};

RerunDisplay::RerunDisplay(bool spawn, const std::string &file, const std::string &address,
                           const std::string &image_encoding, const std::string &hud_encoding)
    : impl_(std::make_unique<Impl>())
{
    if (image_encoding != "jpeg" && image_encoding != "bgr")
        throw std::runtime_error("display.image_encoding must be jpeg or bgr");
    if (hud_encoding != "png" && hud_encoding != "bgra")
        throw std::runtime_error("display.hud_encoding must be png or bgra");

    auto recording = std::make_unique<rerun::RecordingStream>("mas_vision");
    if (spawn && address.empty())
    {
        rerun::SpawnOptions options;
        options.memory_limit        = "2GiB";
        options.hide_welcome_screen = true;
        // 项目环境、共享环境、PATH 依次查找，无需激活虚拟环境。
        auto viewer       = project_root() / ".venv-openvino/bin/rerun";
        const auto shared = shared_venv();
        if (!std::filesystem::is_regular_file(viewer) && !shared.empty())
            viewer = shared / "bin/rerun";
        if (std::filesystem::is_regular_file(viewer))
            options.executable_path = viewer.string();
        try
        {
            recording->spawn(options).throw_on_failure();
        }
        catch (const std::exception &error)
        {
            throw std::runtime_error(
                std::string("Rerun Viewer startup failed. Use display.backend: sdl for local "
                            "images, or install rerun-sdk==0.41.0 into the project/shared environment.\n") +
                error.what());
        }
    }

    const auto url = address.empty() ? "rerun+http://127.0.0.1:9876/proxy" : address;
    if (!file.empty() && (spawn || !address.empty()))
        recording->set_sinks(rerun::FileSink{file}, rerun::GrpcSink{url}).throw_on_failure();
    else if (!file.empty())
        recording->save(file).throw_on_failure();
    else if (!address.empty())
        recording->connect_grpc(address).throw_on_failure();
    else if (!spawn)
        throw std::runtime_error("Rerun needs a Viewer, address or recording file");

    recording->log_static("world", rerun::ViewCoordinates::FLU);
    impl_->recording = std::move(recording);
    impl_->raw_image = image_encoding == "bgr";
    impl_->raw_hud   = hud_encoding == "bgra";
}

RerunDisplay::~RerunDisplay() = default;

void RerunDisplay::flush()
{
    impl_->recording->flush_blocking(5).throw_on_failure();
    MAS_LOG_INFO("Rerun submitted: images={}, hud={}, image_bytes={}, hud_bytes={} (SDK input, not Viewer FPS)",
                 impl_->image_logs, impl_->hud_logs, impl_->image_bytes, impl_->hud_bytes);
}

void RerunDisplay::layout(const std::vector<DisplayView> &views, std::span<const unsigned char> blueprint)
{
    impl_->hud_heights.clear();
    for (const auto &view : views)
        impl_->hud_heights[view.name] = view.hud_height;
    impl_->shared_image = !blueprint.empty();
    if (blueprint.empty())
        return;

    impl_->recording
        ->try_log_file_from_contents("layout.rbl", reinterpret_cast<const std::byte *>(blueprint.data()),
                                     blueprint.size())
        .throw_on_failure();
}

void RerunDisplay::set_time(double seconds)
{
    impl_->recording->set_time_duration_secs("exposure", seconds);

    if (seconds != impl_->frame_time)
    {
        impl_->frame_time = seconds;
        impl_->jpeg.clear();
        impl_->image_logged = false;
    }
}

void RerunDisplay::add(const std::string &name, const cv::Mat &image, const std::vector<DisplayText> &texts,
                       const std::vector<DisplayPoint> &points, const std::vector<DisplayLine> &lines)
{
    const auto root = name + "/image";

    // 共享布局每帧只记录一次底图，图元和 HUD 仍属于各自视图。
    if (!impl_->shared_image || !impl_->image_logged)
        impl_->log_image(root, image);

    impl_->log_geometry(root, points, lines, texts);
    impl_->log_hud(name, root, image, texts);
}

void RerunDisplay::Impl::log_image(const std::string &root, const cv::Mat &image)
{
    auto &rec = *recording;

    if (image.type() != CV_8UC3)
        throw std::runtime_error("Rerun expects an 8-bit BGR image");

    const auto image_path = shared_image ? std::string("camera/image") : root;
    if (raw_image)
    {
        const cv::Mat contiguous = image.isContinuous() ? image : image.clone();
        rec.log(image_path,
                rerun::Image(contiguous.ptr<std::uint8_t>(),
                             {static_cast<std::uint32_t>(image.cols), static_cast<std::uint32_t>(image.rows)},
                             rerun::ColorModel::BGR));
        image_bytes += image.total() * image.elemSize();
    }
    else
    {
        cv::imencode(".jpg", image, jpeg, {cv::IMWRITE_JPEG_QUALITY, 80});
        rec.log(image_path, rerun::EncodedImage::from_bytes(jpeg, "image/jpeg"));
        image_bytes += jpeg.size();
    }

    ++image_logs;
    image_logged = true;
}

void RerunDisplay::Impl::log_geometry(const std::string &root, const std::vector<DisplayPoint> &points,
                                      const std::vector<DisplayLine> &lines, const std::vector<DisplayText> &texts)
{
    auto &rec = *recording;

    std::vector<rerun::Position2D> positions;
    std::vector<rerun::Color> colors;
    std::vector<rerun::Radius> radii;
    for (const auto &point : points)
    {
        positions.emplace_back(float(point.x), float(point.y));
        colors.emplace_back(point.color.r, point.color.g, point.color.b, point.color.a);
        radii.emplace_back(float(point.size));
    }
    // 空数组也提交，清除上一帧的点和线，避免丢失目标后仍显示旧观测。
    rec.log(root + "/points", rerun::Points2D(positions).with_colors(colors).with_radii(radii));

    std::vector<rerun::LineStrip2D> strips;
    colors.clear();
    radii.clear();
    for (const auto &line : lines)
    {
        strips.emplace_back(
            std::vector<rerun::Vec2D>{{float(line.x1), float(line.y1)}, {float(line.x2), float(line.y2)}});
        colors.emplace_back(line.color.r, line.color.g, line.color.b, line.color.a);
        radii.emplace_back(float(line.thickness) / 2);
    }
    rec.log(root + "/lines", rerun::LineStrips2D(strips).with_colors(colors).with_radii(radii));

    positions.clear();
    colors.clear();
    std::vector<std::string> labels;
    for (const auto &text : texts)
    {
        if (text.image_overlay)
            continue;
        positions.emplace_back(float(text.x), float(text.y));
        colors.emplace_back(text.color.r, text.color.g, text.color.b, text.color.a);
        labels.push_back(text.content);
    }
    // 标签附着于像素坐标；字号与布局由 Viewer 管理，不再依赖字体文件。
    rec.log(root + "/labels", rerun::Points2D(positions).with_radii(0.f).with_colors(colors).with_labels(labels));
}

void RerunDisplay::Impl::log_hud(const std::string &name, const std::string &root, const cv::Mat &image,
                                 const std::vector<DisplayText> &texts)
{
    auto &rec = *recording;

    // HUD 只覆盖状态区，与底图使用相同像素原点。
    // 保留真实文字面板，避免点标签居中造成左边缘裁字。
    cv::Size panel_size;
    for (const auto &text : texts)
    {
        if (!text.image_overlay)
            continue;

        int baseline      = 0;
        const auto size   = cv::getTextSize(text.content, cv::FONT_HERSHEY_SIMPLEX, text.size / 24.,
                                            std::max(1, text.thickness), &baseline);
        panel_size.width  = std::max(panel_size.width, text.x + size.width + 10);
        panel_size.height = std::max(panel_size.height, text.y + size.height + baseline + 8);
    }

    if (panel_size.area() > 0)
    {
        // 固定高度由调用方布局提供，避免底层根据业务模块名称猜测文本行数。
        const auto configured = hud_heights.find(name);
        const int hud_height =
            configured != hud_heights.end() && configured->second > 0 ? configured->second : panel_size.height;
        const cv::Size capacity(image.cols, std::min(image.rows, hud_height));
        const double fit =
            std::min({1.0, double(capacity.width) / panel_size.width, double(capacity.height) / panel_size.height});
        panel_size  = {std::max(1, cvFloor(panel_size.width * fit)), std::max(1, cvFloor(panel_size.height * fit))};
        auto &panel = hud_buffers[name];
        panel.create(capacity, CV_8UC4);
        panel.setTo(cv::Scalar(0, 0, 0, 0));
        cv::rectangle(panel, cv::Rect({}, panel_size), cv::Scalar(0, 0, 0, 180), cv::FILLED);
        for (const auto &text : texts)
        {
            if (!text.image_overlay)
                continue;

            int baseline        = 0;
            const double scale  = text.size / 24. * fit;
            const int thickness = std::max(1, cvRound(text.thickness * fit));
            const auto size     = cv::getTextSize(text.content, cv::FONT_HERSHEY_SIMPLEX, scale, thickness, &baseline);
            // 借鉴 RMCS utility/image/text.cpp 的黑色偏移描边，改善明亮背景上的可读性。
            const cv::Point origin(cvRound(text.x * fit), cvRound(text.y * fit) + size.height);
            cv::putText(panel, text.content, origin + cv::Point(1, 1), cv::FONT_HERSHEY_SIMPLEX, scale, {0, 0, 0, 255},
                        thickness + 2, cv::LINE_AA);
            cv::putText(panel, text.content, origin, cv::FONT_HERSHEY_SIMPLEX, scale,
                        {double(text.color.b), double(text.color.g), double(text.color.r), 255}, thickness,
                        cv::LINE_AA);
        }

        if (raw_hud)
        {
            rec.log(root + "/hud",
                    rerun::Image(panel.ptr<std::uint8_t>(),
                                 {static_cast<std::uint32_t>(panel.cols), static_cast<std::uint32_t>(panel.rows)},
                                 rerun::ColorModel::BGRA)
                        .with_draw_order(20.f));
            hud_bytes += panel.total() * panel.elemSize();
        }
        else
        {
            cv::imencode(".png", panel, png, {cv::IMWRITE_PNG_COMPRESSION, 1});
            rec.log(root + "/hud", rerun::EncodedImage::from_bytes(png, "image/png").with_draw_order(20.f));
            hud_bytes += png.size();
        }
        ++hud_logs;
    }
    else
        rec.log(root + "/hud", rerun::Clear::RECURSIVE);
}

void RerunDisplay::scalar(const std::string &path, double value)
{
    if (std::isfinite(value))
        impl_->recording->log(path, rerun::Scalars(value));
    else
        // 当前曲线在此时刻失效；保留历史，并且不清除子路径。
        impl_->recording->log(path, rerun::Clear::FLAT);
}

void RerunDisplay::points(const std::string &path, const std::vector<cv::Point2d> &points,
                          const std::vector<std::string> &labels)
{
    std::vector<rerun::Position2D> positions;

    for (const auto &point : points)
        positions.emplace_back(float(point.x), float(point.y));
    impl_->recording->log(
        path, rerun::Points2D(positions).with_radii(3.f).with_colors(rerun::Color(255, 80, 255)).with_labels(labels));
}

void RerunDisplay::point3d(const std::string &path, double x, double y, double z)
{
    std::vector<rerun::Position3D> positions;
    if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z))
        positions.emplace_back(float(x), float(y), float(z));
    impl_->recording->log(path, rerun::Points3D(positions).with_radii(.02f));
}

} // namespace rm_utils
