#include "usbcamera.hpp"
#include "mas_log.hpp"
#include "usbcamera_config.hpp"
#include <opencv2/videoio.hpp>
#include <algorithm>
#include <cctype>

namespace hardware
{

UsbCamera::UsbCamera(const std::string &device_path) : device_path_(device_path), opened_(false)
{
}

UsbCamera::~UsbCamera()
{
    closeCamera();
}

bool UsbCamera::openCamera()
{
    if (isConnectedStatus())
    {
        return true;
    }

    opened_ = false;
    last_reconnect_ = std::chrono::steady_clock::now();
    try
    {
        // 使用设备路径打开相机
        // V4L2 支持设备索引字符串和 /dev/videoN 路径。
        const bool index = !device_path_.empty() && std::all_of(device_path_.begin(), device_path_.end(),
            [](unsigned char ch) { return std::isdigit(ch); });
        const int backend = cv::CAP_V4L2;
        if (index) cap_.open(std::stoi(device_path_), backend);
        else cap_.open(device_path_, backend);

        if (!cap_.isOpened())
        {
            MAS_LOG_ERROR("Could not open USB camera. Path: {}",
                            device_path_.c_str());
            return false;
        }

        if (cap_.isOpened())
        {
            cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

            opened_ = true;

            applyConfig(UsbCameraConfigManager::getWidth(), UsbCameraConfigManager::getHeight(),
                        UsbCameraConfigManager::getFps(), UsbCameraConfigManager::getAutoExposure(),
                        UsbCameraConfigManager::getExposure(), UsbCameraConfigManager::getGain(),
                        UsbCameraConfigManager::getAutoWb());

            MAS_LOG_INFO("Successfully opened USB camera. Path: {}", device_path_.c_str());

            return true;
        }
    }
    catch (const std::exception &e)
    {
        MAS_LOG_ERROR("Exception opening USB camera: {}", e.what());
        closeCamera();
        return false;
    }

    return false;
}

void UsbCamera::closeCamera()
{
    opened_ = false;
    if (cap_.isOpened())
    {
        cap_.release();
        MAS_LOG_INFO("USB camera closed");
    }
}

bool UsbCamera::applyConfig(int width, int height, int fps, int auto_exposure, int exposure,
                            int gain, int auto_wb)
{
    if (!isConnectedStatus())
    {
        MAS_LOG_WARN("Cannot apply config when camera is not opened");
        return false;
    }

    bool success = true;

    success = cap_.set(cv::CAP_PROP_FRAME_WIDTH, width) && success;
    success = cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height) && success;
    success = cap_.set(cv::CAP_PROP_FPS, fps) && success;
    success = cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, auto_exposure) && success;
    success = cap_.set(cv::CAP_PROP_EXPOSURE, exposure) && success;
    success = cap_.set(cv::CAP_PROP_GAIN, gain) && success;
    success = cap_.set(cv::CAP_PROP_AUTO_WB, auto_wb) && success;

    if (!success) MAS_LOG_WARN("Some USB controls were rejected by the camera driver; verify actual settings");
    return success;
}

CameraFrame UsbCamera::getImage()
{
    if (!isConnectedStatus())
    {
        const auto now = std::chrono::steady_clock::now();
        if (now - last_reconnect_ < std::chrono::seconds(1)) return {};
        last_reconnect_ = now;
        if (!openCamera()) return {};
    }

    try
    {
        cv::Mat frame;
        bool    ret = cap_.read(frame);
        if (!ret || frame.empty()) {
            closeCamera();
            return {};
        }
        const auto received = std::chrono::steady_clock::now();
        // OpenCV exposure 控制值的单位依驱动变化；时间补偿单独使用明确的微秒/毫秒参数。
        const auto delay = std::chrono::duration<double>(
            UsbCameraConfigManager::getExposureTimeUs() * 0.5e-6 +
            UsbCameraConfigManager::getTransferLatencyMs() * 1e-3);
        return CameraFrame{frame, received - std::chrono::duration_cast<std::chrono::steady_clock::duration>(delay), received};
    }
    catch (const std::exception &e)
    {
        MAS_LOG_ERROR("Exception capturing image: {}", e.what());
        closeCamera();
        return CameraFrame{};
    }
}

std::string UsbCamera::getDevicePath() const
{
    return device_path_;
}

} // namespace hardware
