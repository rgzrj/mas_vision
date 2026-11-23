#pragma once
#include <opencv2/core.hpp>
#include "auto_buff/buff_types.hpp"

#include <opencv2/core/mat.hpp>

namespace auto_buff {

// 原版 RMCS 传统单帧检测；本地 TraditionalDetector 负责对外观测转换和颜色切换。
class RuneDetector {
public:
    struct Config {
        cv::Mat intrinsic, distortion; // 内参/畸变用于把物理尺寸及距离范围换算为像素过滤条件。
        auto_buff::EnemyColor color = auto_buff::EnemyColor::BLUE; // 本次检测只处理一种颜色。

        double max_distance = 5.0; // 最远检测距离，米。
        double min_distance = 1.7; // 最近检测距离，米。

        double max_perspective = 60.; // 允许的最大透视倾斜角，度，用于椭圆轴比过滤。

        // 激活靶心判定阈值，使用 PVD（Peak-Valley Depth）指标：
        // PVD ≈ 0 表示角度投影各向均匀，为击打后已激活的圆环；
        // PVD 较大表示存在清晰的 4 个峰/谷结构，为尚未击打的完整符板。
        // PVD ≥ 0.2 时按 4 臂端点提取，PVD < 0.2 时退化为圆环处理。
        double active_threshold = 0.2;

        // R 标模板匹配阈值。采用骨架拓扑特征匹配，值越大表示 ROI 越像 R 标；
        // 分数 < 阈值时直接丢弃该候选。
        double match_threshold = 0.50;
    } config;

    struct Elements {
        std::vector<auto_buff::RuneBullseye> bullseyes; // 原图坐标的符板中心、四点及 PVD 外观状态。
        std::vector<auto_buff::RuneIcon> icons;        // 通过骨架拓扑评分的全部 R 候选。
    };

    /**
     * @brief 颜色分割、物理尺寸过滤、四臂/圆环判别及 R 骨架识别。
     * @param image 原始 BGR 图像；内部 ROI 结果在返回前恢复为原图坐标。
     * @return 全部有效候选；单帧无状态，不负责关联、预测或 PnP。
     */
    auto detect(const cv::Mat& image) const -> Elements;
};

}
