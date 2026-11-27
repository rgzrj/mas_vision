#include "tests/common/detection.hpp"
#include "auto_buff/buff_detect/yolo/timing.hpp"
#include "auto_buff/buff_detect/yolo/yolo.hpp"
#include "auto_buff/buff_detect/yolo/observation/pose_output.hpp"
#include "auto_buff/buff_detect/yolo/pixels/pixel_geometry.hpp"
#include "auto_buff/buff_detect/yolo/pixels/pixel_refiner.hpp"
#include "rm_utils/io/media_reader.hpp"
#include "tests/common/test.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <numbers>
#include <limits>
#include <sstream>

using namespace auto_buff;
namespace fs = std::filesystem;
using tests::require;

namespace
{
void check_pose_decode()
{
    using namespace auto_buff;
    using tests::require;
    constexpr std::size_t count = 4;
    std::vector<float> raw(18 * count);
    const std::array<cv::Point2d, 5> points{{{100, 50}, {50, 100}, {100, 300}, {150, 100}, {100, 150}}};
    for (std::size_t n = 0; n < count; ++n)
    {
        raw[(n == 3 ? 2 : 0) * count + n] = n == 1 ? .7f : .9f;
        for (int k = 0; k < 5; ++k)
        {
            const auto p                 = points[k] + (n == 2 ? cv::Point2d(220, 0) : cv::Point2d{});
            raw[(3 + k * 3) * count + n] = float(p.x * .5 + 10);
            raw[(4 + k * 3) * count + n] = float(p.y * .5 + 20);
            raw[(5 + k * 3) * count + n] = .95f;
        }
    }
    const auto decoded = auto_buff::decode_pose_output(raw, count, {500, 400}, .5, {10, 20}, .5, .8, 30);
    require(decoded.size() == 3, "SZU NMS merged two boards sharing an R or mixed activation groups");
    require(cv::norm(decoded[0].points[2] - points[2]) < 1e-6, "SZU R/letterbox indices are wrong");
    // A low-score endpoint remains a search/NMS prior, never a usable measurement.
    std::vector<float> duplicate_raw(18 * 2, 0.f);
    const std::array<cv::Point2d, 5> duplicate_points{{{100, 0}, {0, 100}, {100, 300}, {200, 100}, {100, 200}}};
    for (int n = 0; n < 2; ++n)
    {
        duplicate_raw[n] = n ? .7f : .9f;
        for (int k = 0; k < 5; ++k)
        {
            duplicate_raw[(3 + 3 * k) * 2 + n] = float(duplicate_points[k].x);
            duplicate_raw[(4 + 3 * k) * 2 + n] = float(duplicate_points[k].y);
            duplicate_raw[(5 + 3 * k) * 2 + n] = k == 0 && n == 0 ? .1f : .95f;
        }
    }
    const auto suppressed = decode_pose_output(duplicate_raw, 2, {500, 400}, 1, {}, .5, .8, 30);
    require(suppressed.size() == 1 && !suppressed[0].valid[0] && suppressed[0].points[0] == duplicate_points[0],
            "A low-score point shifted the NMS center or was promoted to a usable endpoint");
    cv::Mat image(400, 500, CV_8UC3, cv::Scalar());
    for (const auto &pose : decoded)
        for (int k : {0, 1, 3, 4})
            cv::circle(image, pose.points[k], 5, {0, 0, 255}, -1);
    auto shared_r         = decoded;
    shared_r[1].points[2] = points[2];
    auto result           = auto_buff::make_pose_detection(shared_r, image, auto_buff::EnemyColor::RED, false);
    require(result.markers.size() == 1 && result.boards.size() == 2,
            "SZU shared R was duplicated or activated arm became a board");
    require(result.boards.front().corners == std::array<cv::Point2d, 4>{points[0], points[1], points[4], points[3]},
            "YOLO board corner order is not top/left/bottom/right");
    require(result.light_arms.size() == 1 && result.light_arms[0].pose.points == shared_r.back().points,
            "Big-activated raw points were discarded or refined while disabled");
    const auto enabled_arm = make_pose_detection({shared_r.back()}, image, auto_buff::EnemyColor::RED, true);
    require(enabled_arm.boards.empty() && enabled_arm.light_arms.size() == 1 &&
                enabled_arm.light_arms[0].pose.points == shared_r.back().points,
            "Board refinement changed big-activated raw arm points");
    require(make_pose_detection({shared_r.back()}, image, auto_buff::EnemyColor::BLUE, false).light_arms.empty(),
            "Big-activated color filtering failed");
    auto partial_arm        = shared_r.back();
    partial_arm.valid[0]    = false;
    partial_arm.points[0].x = std::numeric_limits<double>::quiet_NaN();
    auto partial_detection  = make_pose_detection({partial_arm}, image, auto_buff::EnemyColor::RED, true);
    require(partial_detection.light_arms.size() == 1 && partial_detection.boards.empty(),
            "Partial light-arm diagnostics were discarded or converted to a board");
    require(result.markers[0].color == auto_buff::EnemyColor::UNKNOWN, "YOLO R inherited the configured color filter");
    require(result.markers[0].observed && result.markers[0].score == shared_r.front().score,
            "YOLO R lost its observation flag or target score");
    require(result.boards[0].features_valid && cv::norm(result.boards[0].center - cv::Point2d(100, 100)) < 1e-6,
            "SZU board center is not the intersection of physical diagonals");
    require(auto_buff::make_pose_detection(shared_r, image, auto_buff::EnemyColor::BLUE, false).boards.empty(),
            "SZU ignored color switching");
    auto partial     = shared_r[0];
    partial.valid[0] = false;
    require(auto_buff::make_pose_detection({partial}, image, auto_buff::EnemyColor::RED, false).boards.empty(),
            "SZU manufactured a raw center from incomplete points");
    auto small  = shared_r[0];
    small.state = 1;
    auto active = auto_buff::make_pose_detection({small}, image, auto_buff::EnemyColor::RED, false);
    require(active.boards.size() == 1 && !active.boards[0].features_valid &&
                active.boards[0].appearance == auto_buff::Appearance::single_ring,
            "SZU activated state became an initialization seed");
    auto mixed_classes = result;
    mixed_classes.boards.push_back(active.boards[0]);
    mixed_classes.raw_poses = shared_r;
    auto small_mode         = mixed_classes;
    filter_rune_observations(small_mode, RuneMode::small, auto_buff::EnemyColor::RED);
    require(small_mode.boards.size() == 3 && small_mode.light_arms.empty() &&
                small_mode.raw_poses.size() == shared_r.size(),
            "Small mode lost pending/ring observations or raw three-class diagnostics");
    auto big_mode = mixed_classes;
    filter_rune_observations(big_mode, RuneMode::big, auto_buff::EnemyColor::RED);
    require(big_mode.boards.size() == 2 && big_mode.light_arms.size() == 1 &&
                big_mode.raw_poses.size() == shared_r.size(),
            "Big mode lost coexisting pending/arm observations or retained the small ring");
    filter_rune_observations(mixed_classes, RuneMode::unknown, auto_buff::EnemyColor::RED);
    require(mixed_classes.boards.size() == 3 && mixed_classes.light_arms.size() == 1,
            "Unknown offline mode discarded an observation class");
    raw[5 * count]     = std::numeric_limits<float>::quiet_NaN();
    const auto invalid = auto_buff::decode_pose_output(raw, count, {500, 400}, .5, {10, 20}, .5, .8, 30);
    require(!invalid.empty() && !invalid[0].valid[0], "SZU accepted a NaN keypoint probability");
    std::cout << "SZU: 18-channel decode, point order, shared R, class geometry and missing points passed.\n";
}
} // namespace

void check_yolo(bool run_model)
{
    check_pose_decode();
    const auto root = tool::from_utf8(BUFF_PROJECT_DIR);
    // Fixed physical pixels with deliberately jittering YOLO boxes. The four
    // measured arms have unequal angular spacing and a foreshortened ellipse.
    cv::Mat pattern(280, 320, CV_8UC3, cv::Scalar(0));
    const cv::Point2d true_center(160, 140);
    auto physical = [&](double angle, double radius)
    {
        auto d   = cv::Point2d(62 * std::cos(angle) * radius, 43 * std::sin(angle) * radius);
        double a = .35;
        return true_center + cv::Point2d(d.x * std::cos(a) - d.y * std::sin(a), d.x * std::sin(a) + d.y * std::cos(a));
    };
    cv::ellipse(pattern, true_center, {62, 43}, .35 * 180 / std::numbers::pi, 0, 360, {0, 0, 220}, 3, cv::LINE_AA);
    cv::ellipse(pattern, true_center, {34, 24}, .35 * 180 / std::numbers::pi, 0, 360, {0, 0, 220}, 3, cv::LINE_AA);
    std::array<cv::Point2d, 4> true_tips;
    for (int k = 0; k < 4; ++k)
    {
        double angle = (20 + k * 90 + (k % 2 ? 8 : -5)) * std::numbers::pi / 180;
        true_tips[k] = physical(angle, 1.12);
        cv::line(pattern, physical(angle, .54), true_tips[k], {0, 0, 255}, 5, cv::LINE_AA);
        cv::circle(pattern, true_tips[k], 2, {255, 255, 255}, cv::FILLED, cv::LINE_AA);
    }
    PoseDetection raw_pose;
    raw_pose.valid.fill(true);
    raw_pose.point_scores.fill(.95);
    raw_pose.points[2] = {40, 240};
    for (int k = 0; k < 4; ++k)
        raw_pose.points[std::array{0, 1, 4, 3}[k]] = true_tips[k];
    const auto refine_pattern = [&](const cv::Mat &image, Board &board)
    { return refine_yolo_board_pixels(image, board, yolo_board_contour(image, raw_pose, board.color)); };
    std::vector<cv::Point2d> recovered;
    for (const auto &shift : std::array<cv::Point2d, 5>{{{0, 0}, {4, -3}, {-4, 3}, {3, 4}, {-3, -4}}})
    {
        auto_buff::Board board;
        board.center        = true_center + shift;
        board.radius        = 58 + shift.x;
        board.color         = auto_buff::EnemyColor::RED;
        board.appearance    = auto_buff::Appearance::full_mark;
        bool found_geometry = refine_pattern(pattern, board);
        if (!found_geometry || !board.features_valid)
            std::cerr << "Geometry diagnostic: fit=" << found_geometry << " pvd=" << board.pvd
                      << " center=" << board.center << " radius=" << board.radius << '\n';
        require(found_geometry && board.features_valid, "Jittered YOLO ROI lost the measured four-arm pattern");
        require(cv::norm(board.center - true_center) < 1.8, "Pixel fit kept the jittering box center");
        for (auto tip : true_tips)
            require(std::ranges::any_of(board.corners, [&](auto p) { return cv::norm(p - tip) < 5; }),
                    "Independent physical arm endpoints were replaced with orthogonal "
                    "rays");
        recovered.push_back(board.center);
    }
    for (auto center : recovered)
        require(cv::norm(center - recovered[0]) < 1.5, "Box jitter still moves the fitted center");
    auto disturbed = pattern.clone();
    cv::circle(disturbed, {235, 140}, 4, {255, 255, 255}, cv::FILLED, cv::LINE_AA);
    Board isolated;
    isolated.center     = true_center;
    isolated.radius     = 58;
    isolated.color      = auto_buff::EnemyColor::RED;
    isolated.appearance = Appearance::full_mark;
    require(refine_pattern(disturbed, isolated) && isolated.features_valid &&
                cv::norm(isolated.center - recovered[0]) < 1.,
            "A disconnected bright spot changed the target geometry");
    auto occluded = pattern.clone();
    cv::rectangle(occluded, {160, 0, 160, 280}, cv::Scalar(0), cv::FILLED);
    require(!refine_pattern(occluded, isolated) && !isolated.features_valid,
            "An incomplete target contour was accepted as a complete plate");
    auto_buff::Board absent;
    absent.center = true_center;
    absent.radius = 58;
    absent.color  = auto_buff::EnemyColor::RED;
    require(!refine_pattern(cv::Mat::zeros(pattern.size(), pattern.type()), absent),
            "Blank ROI acquired pixel geometry");
    cv::Mat circle = cv::Mat::zeros(pattern.size(), pattern.type());
    cv::ellipse(circle, true_center, {62, 43}, .35 * 180 / std::numbers::pi, 0, 360, {0, 0, 220}, 3, cv::LINE_AA);
    absent.appearance = auto_buff::Appearance::full_mark;
    require(!refine_pattern(circle, absent) && !absent.features_valid,
            "A single ellipse manufactured a physical center or pose endpoints");
    cv::Mat incomplete = circle.clone();
    cv::ellipse(incomplete, true_center, {34, 24}, .35 * 180 / std::numbers::pi, 0, 360, {0, 0, 220}, 3, cv::LINE_AA);
    for (int k = 0; k < 3; ++k)
    {
        double angle = (20 + k * 90 + (k % 2 ? 8 : -5)) * std::numbers::pi / 180;
        cv::line(incomplete, physical(angle, .54), true_tips[k], {0, 0, 255}, 5, cv::LINE_AA);
    }
    auto_buff::Board incomplete_board;
    incomplete_board.center     = true_center;
    incomplete_board.radius     = 58;
    incomplete_board.color      = auto_buff::EnemyColor::RED;
    incomplete_board.appearance = auto_buff::Appearance::full_mark;
    require(refine_pattern(incomplete, incomplete_board) && !incomplete_board.features_valid &&
                cv::norm(incomplete_board.center - true_center) < 1.8 &&
                std::count(incomplete_board.corners.begin(), incomplete_board.corners.end(), cv::Point2d{}) == 1,
            "Three measured arms lost their physical center or manufactured a fourth endpoint");
    auto_buff::Board original;
    original.center         = true_center;
    original.radius         = 62;
    original.corners        = true_tips;
    original.features_valid = true;
    original.color          = auto_buff::EnemyColor::RED;
    original.appearance     = auto_buff::Appearance::full_mark;
    require(
        auto_buff::refine_yolo_pose_pixels(cv::Mat::zeros(pattern.size(), pattern.type()), raw_pose, original, true) &&
            original.features_valid && original.center == true_center && original.corners == true_tips,
        "Failed pixel refinement discarded reliable raw points");
    require(
        !auto_buff::refine_yolo_pose_pixels(cv::Mat::zeros(pattern.size(), pattern.type()), raw_pose, original, false),
        "Invalid raw center fell back to a box center");
    for (const auto &pixels : {circle, incomplete})
    {
        auto fallback = original;
        fallback.center += cv::Point2d(3, -2);
        const auto center_before = fallback.center;
        require(refine_yolo_pose_pixels(pixels, raw_pose, fallback, true) && fallback.center == center_before &&
                    fallback.corners == original.corners && fallback.features_valid,
                "Partial refinement mixed an ellipse center with raw endpoints");
        fallback.features_valid     = false;
        const bool recovered_center = refine_yolo_pose_pixels(pixels, raw_pose, fallback, false);
        require(pixels.data == circle.data
                    ? !recovered_center
                    : recovered_center && !fallback.features_valid && cv::norm(fallback.center - true_center) < 1.8,
                "Partial refinement confused an ellipse with a center measured from real axes");
    }
    raw_pose.valid[0]        = false;
    raw_pose.point_scores[0] = .1;
    require(make_pose_detection({raw_pose}, pattern, auto_buff::EnemyColor::RED, false).boards.empty(),
            "A low-score raw endpoint became usable without pixel evidence");
    const auto pixel_full = make_pose_detection({raw_pose}, pattern, auto_buff::EnemyColor::RED, true);
    require(pixel_full.boards.size() == 1 && pixel_full.boards[0].features_valid &&
                cv::norm(pixel_full.boards[0].center - true_center) < 1.8,
            "Semantic contour refinement failed to recover real endpoints with a low-score search prior");
    const auto pixel_partial = make_pose_detection({raw_pose}, incomplete, auto_buff::EnemyColor::RED, true);
    require(pixel_partial.boards.size() == 1 && !pixel_partial.boards[0].features_valid &&
                cv::norm(pixel_partial.boards[0].center - true_center) < 1.8,
            "A center-only pixel observation was discarded or allowed to seed PnP");
    raw_pose.points[2] = true_center;
    require(make_pose_detection({raw_pose}, pattern, auto_buff::EnemyColor::RED, true).boards.empty(),
            "A contour enclosing R was accepted as a plate");
    // A real perspective projection, rather than affine ellipse scaling. Its ellipse center
    // is several pixels away from the physical center projection at the closer distance.
    for (double depth : {3., 1.5})
    {
        cv::Mat perspective(800, 1000, CV_8UC3, cv::Scalar(0));
        cv::Mat intrinsic = (cv::Mat_<double>(3, 3) << 1600, 0, 500, 0, 1600, 400, 0, 0, 1);
        const cv::Vec3d rotation(0, 35 * std::numbers::pi / 180, 0), translation(0, 0, depth);
        const auto project = [&](const std::vector<cv::Point3f> &model)
        {
            std::vector<cv::Point2f> result;
            cv::projectPoints(model, rotation, translation, intrinsic, cv::Mat(), result);
            return result;
        };
        std::vector<cv::Point3f> model;
        for (int k = 0; k < 720; ++k)
        {
            double angle = k * 2 * std::numbers::pi / 720;
            model.emplace_back(float(.15 * std::cos(angle)), float(.15 * std::sin(angle)), 0);
        }
        std::vector<cv::Point> ring_pixels;
        for (auto p : project(model))
            ring_pixels.emplace_back(cvRound(p.x), cvRound(p.y));
        cv::polylines(perspective, ring_pixels, true, {0, 0, 220}, 3, cv::LINE_AA);
        cv::Mat three_arm_perspective;
        std::vector<cv::Point2f> projected_tips;
        for (int k = 0; k < 4; ++k)
        {
            double angle = (20 + 90 * k) * std::numbers::pi / 180;
            auto arm     = project({{float(.08 * std::cos(angle)), float(.08 * std::sin(angle)), 0},
                                    {float(.168 * std::cos(angle)), float(.168 * std::sin(angle)), 0}});
            cv::line(perspective, arm[0], arm[1], {0, 0, 255}, 5, cv::LINE_AA);
            if (k == 2)
                three_arm_perspective = perspective.clone();
            projected_tips.push_back(arm[1]);
        }
        Board measured;
        measured.center     = {503, 398};
        measured.radius     = std::min(cv::boundingRect(ring_pixels).width, cv::boundingRect(ring_pixels).height) / 2.;
        measured.color      = auto_buff::EnemyColor::RED;
        measured.appearance = Appearance::full_mark;
        PoseDetection projected_pose;
        projected_pose.valid.fill(true);
        projected_pose.points[2] = {100, 100};
        for (int k = 0; k < 4; ++k)
            projected_pose.points[std::array{0, 1, 4, 3}[k]] = projected_tips[k];
        bool refined = refine_yolo_board_pixels(perspective, measured,
                                                yolo_board_contour(perspective, projected_pose, measured.color));
        std::cout << "Perspective geometry: depth=" << depth << " refined=" << refined
                  << " features=" << measured.features_valid << " center=" << measured.center << '\n';
        require(refined && measured.features_valid && cv::norm(measured.center - cv::Point2d(500, 400)) < 2.,
                "Perspective refinement used the ellipse center as the physical center");
        for (auto tip : projected_tips)
            require(std::ranges::any_of(measured.corners, [&](auto p) { return cv::norm(p - cv::Point2d(tip)) < 5.; }),
                    "Perspective refinement moved endpoints away from measured arms");
        require(refine_yolo_board_pixels(three_arm_perspective, measured,
                                         yolo_board_contour(three_arm_perspective, projected_pose, measured.color)) &&
                    !measured.features_valid && cv::norm(measured.center - cv::Point2d(500, 400)) < 2. &&
                    std::count(measured.corners.begin(), measured.corners.end(), cv::Point2d{}) == 1,
                "Three perspective arms used the ellipse center or manufactured a missing endpoint");
    }
    cv::Mat mixed(200, 200, CV_8UC3, cv::Scalar());
    cv::circle(mixed, {100, 100}, 48, {0, 0, 220}, 3);
    cv::rectangle(mixed, {40, 40, 120, 120}, {255, 0, 0}, 3);
    // 模型框稍大时，外侧蓝灯会进入椭圆ROI；边缘降权应保留内侧真实红环的颜色。
    cv::ellipse(mixed, {100, 100}, {58, 58}, 0, 0, 75, {255, 0, 0}, 6);
    cv::ellipse(mixed, {100, 100}, {58, 58}, 0, 180, 255, {255, 0, 0}, 6);
    auto_buff::Board ring;
    ring.center     = {100, 100};
    ring.appearance = auto_buff::Appearance::single_ring;
    require(auto_buff::yolo_board_color(mixed, ring, {40, 40, 120, 120}) == auto_buff::EnemyColor::RED,
            "Blue outer structure changed the red ring color");
    auto_buff::Board blue_disk;
    blue_disk.appearance = auto_buff::Appearance::full_mark;
    blue_disk.center     = {100, 100};
    blue_disk.corners    = {{{100, 35}, {35, 100}, {100, 165}, {165, 100}}};
    mixed.setTo(cv::Scalar());
    cv::circle(mixed, {100, 100}, 20, {0, 0, 255}, 3);
    for (auto point : blue_disk.corners)
        cv::circle(mixed, point, 5, {255, 0, 0}, -1);
    require(auto_buff::yolo_board_color(mixed, blue_disk, {20, 20, 160, 160}) == auto_buff::EnemyColor::BLUE,
            "Red inner light changed the blue disk color");
    std::cout << "YOLO geometry: box jitter, oblique nonorthogonal arms and "
                 "empty/single-ring ROIs passed.\n";
    if (!run_model)
        return; // TensorRT 构建仍运行几何与像素精修检查，不加载 ONNX。

    auto image = tool::read_image(root / "tests/data/img/szu_reference.jpg");
    tests::TemporaryDirectory temporary;
    const auto config_path     = temporary.path / tool::from_utf8("YOLO 配置.yaml");
    const auto model_directory = temporary.path / tool::from_utf8("模型 文件");
    fs::create_directory(model_directory);
    const auto local_model = model_directory / "rune.onnx";
    fs::copy_file(root / "auto_buff/buff_detect/yolo/model/rune_szu.onnx", local_model);
    const auto write_config = [&](const std::string &settings)
    {
        YAML::Node config;
        config["auto_buff"]["detect"]["yolo"] = YAML::Load(settings);
        config["auto_buff"]["detect"]["common"]["color"] = config["auto_buff"]["detect"]["yolo"]["color"].as<std::string>("red");
        config["auto_buff"]["detect"]["common"]["debug"] = config["auto_buff"]["detect"]["yolo"]["debug"].as<bool>(false);
        config["auto_buff"]["detect"]["yolo"]["model"] = tool::utf8(local_model);
        std::ofstream(config_path) << config;
    };
    write_config("color: red\nfeature_points: none");
    auto_buff::YoloDetector yolo(YAML::LoadFile(config_path.string()), config_path.parent_path());
    const auto neural = tests::detect(yolo, image, 0);
    require(!neural.markers.empty(), "SZU model failed to emit its R keypoint");
    require(std::ranges::any_of(neural.boards, [](const auto &board) { return board.features_valid; }),
            "SZU reference frame lost all complete inactive boards");
    for (const auto &board : neural.boards)
        require(board.center.x >= 0 && board.center.x < image.cols && board.center.y >= 0 &&
                    board.center.y < image.rows,
                "SZU letterbox restoration escaped the original image");
    for (const auto color : {auto_buff::EnemyColor::BLUE, auto_buff::EnemyColor::RED})
    {
        yolo.BuffDetector_Set_Color(color);
        const auto switched = tests::detect(yolo, image, 0);
        require(std::ranges::all_of(switched.boards, [&](const auto &board) { return board.color == color; }),
                "YOLO runtime color selection retained the other color");
    }
    write_config("color: red\nfeature_points: rmcs_pixels");
    auto_buff::YoloDetector refined(YAML::LoadFile(config_path.string()), config_path.parent_path());
    const auto plain_result = tests::detect(refined, image, 0);
    require(!plain_result.raw_poses.empty(), "Disabled debug discarded the original three-class points");
    require(!plain_result.boards.empty(), "Pixel refinement discarded every SZU reference observation");
    require(!tracy::IsProfilerStarted() && auto_buff::yolo_timing.yolo_ms == 0 &&
                auto_buff::yolo_timing.refine_calls == 0,
            "Disabled tracy_debug initialized the client or collected timings");
    write_config("color: red\nfeature_points: rmcs_pixels\ntracy_debug: true");
    auto_buff::YoloDetector traced(YAML::LoadFile(config_path.string()), config_path.parent_path());
    const auto unstarted_result = tests::detect(traced, image, 0);
    require(!tracy::IsProfilerStarted() && auto_buff::yolo_timing.yolo_ms == 0 &&
                auto_buff::yolo_timing.refine_calls == 0 && unstarted_result.boards.size() == plain_result.boards.size(),
            "Detector started Tracy or measured before the test initialized the client");
    auto_buff::start_tracy();
    const auto traced_result = tests::detect(traced, image, 0);
    require(tracy::IsProfilerStarted() && auto_buff::yolo_timing.yolo_ms > 0 && auto_buff::yolo_timing.refine_ms > 0 &&
                auto_buff::yolo_timing.refine_calls > 0,
            "Enabled tracy_debug did not collect both YOLO and refinement");
    require(plain_result.boards.size() == traced_result.boards.size() &&
                plain_result.markers.size() == traced_result.markers.size(),
            "tracy_debug changed the detection count");
    for (std::size_t i = 0; i < plain_result.boards.size(); ++i)
        require(cv::norm(plain_result.boards[i].center - traced_result.boards[i].center) < 1e-4 &&
                    plain_result.boards[i].corners == traced_result.boards[i].corners,
                "tracy_debug changed the refined geometry");
    tests::detect(refined, image, 0);
    require(tracy::IsProfilerStarted() && auto_buff::yolo_timing.yolo_ms == 0 && auto_buff::yolo_timing.refine_calls == 0,
            "Started Tracy ignored the detector's disabled collection flag");
    write_config("color: red\nfeature_points: rmcs_pixels\ndebug: true");
    auto_buff::YoloDetector diagnostic(YAML::LoadFile(config_path.string()), config_path.parent_path());
    const auto snapshot = tests::detect(diagnostic, image, 0);
    require(!snapshot.raw_poses.empty(), "Enabled debug lost raw five-point diagnostics");
    const auto saved_points = snapshot.raw_poses.front().points;
    tests::detect(diagnostic, cv::Mat::zeros(image.size(), CV_8UC3), .01);
    require(snapshot.raw_poses.front().points == saved_points, "Inference reused a diagnostic snapshot buffer");
    require(snapshot.boards.size() == plain_result.boards.size() &&
                cv::norm(snapshot.boards.front().center - plain_result.boards.front().center) < 1e-4,
            "Raw debug diagnostics changed refined geometry");
    bool rejected = false;
    try
    {
        write_config("feature_points: invalid");
        auto_buff::YoloDetector invalid(YAML::LoadFile(config_path.string()), config_path.parent_path());
    }
    catch (const std::runtime_error &)
    {
        rejected = true;
    }
    require(rejected, "Unknown YOLO refinement method was accepted");
    std::cout << "SZU reference: " << neural.markers.size() << " R candidates, " << neural.boards.size()
              << " boards; native 640x480 and Chinese model/config paths passed.\n";
}
