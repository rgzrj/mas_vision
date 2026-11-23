#pragma once

#include "auto_buff/buff_types.hpp"
#include <opencv2/core/mat.hpp>

namespace auto_buff {

auto extract_channel(const cv::Mat& src, EnemyColor camp, cv::Mat& dst) -> void;

}
