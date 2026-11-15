#include "session.hpp"
#include "rm_utils/io/media_reader.hpp"
#include <fstream>
#include <stdexcept>
namespace app
{
// 把一套算法的配置和资源复制到录制目录，回放时沿用录制当时的模型与标定。
// copy_model=false仅保存模型绝对路径；XML模型按同名规则同时保存BIN权重。
void save_recording_snapshot(const std::filesystem::path &path, const YAML::Node &config,
                             const std::filesystem::path &output, bool copy_model)
{
    namespace fs = std::filesystem;
    fs::create_directories(output);
    auto saved = YAML::Clone(config);
    for (const auto *name : {"hikcamera.yaml", "serial_config.yaml"})
        if (!fs::exists(output / name))
            fs::copy_file(path.parent_path() / name, output / name);
    const auto copy_resource = [&](YAML::Node field, const std::string &name)
    {
        const auto source = fs::absolute(path.parent_path() / tool::from_utf8(field.as<std::string>()));
        if (copy_model)
        {
            fs::copy_file(source, output / name, fs::copy_options::overwrite_existing);
            if (source.extension() == ".xml")
            {
                auto bin = source;
                bin.replace_extension(".bin");
                auto target = output / name;
                target.replace_extension(".bin");
                fs::copy_file(bin, target, fs::copy_options::overwrite_existing);
            }
            field = name;
        }
        else
            field = tool::utf8(source);
    };
    const bool armor = bool(saved["auto_aim"]);
    if (armor)
    {
        auto numbers = saved["auto_aim"]["armor_detector"]["number_params"];
        copy_resource(numbers["model_path"], "armor_number.onnx");
        copy_resource(numbers["label_path"], "armor_labels.txt");
    }
    else if (saved["auto_buff"]["detect"]["backend"].as<std::string>("traditional") == "yolo")
    {
        auto field = saved["auto_buff"]["detect"]["model"];
        copy_resource(field, fs::path(field.as<std::string>()).extension() == ".xml" ? "model.xml" : "model.onnx");
    }
    std::ofstream stream(output / (armor ? "auto_aim.yaml" : "auto_buff.yaml"));
    stream << saved;
    stream.close();
    if (!stream)
        throw std::runtime_error("Cannot save session configuration");
}
// 独立录制器统一保存armor/buff两套配置；公共设备配置只复制一次。
void save_session_configs(const std::filesystem::path &config_dir, const std::filesystem::path &output)
{
    for (const auto *name : {"auto_aim.yaml", "auto_buff.yaml"})
    {
        const auto path = config_dir / name;
        save_recording_snapshot(path, YAML::LoadFile(path.string()), output, true);
    }
}
} // namespace app
