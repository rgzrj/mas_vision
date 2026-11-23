#ifndef _ARMOR_DETECTOR_H_
#define _ARMOR_DETECTOR_H_

#include <chrono>
#include <map>
#include <opencv2/core/mat.hpp>
#include <string>

#include "armor_types.hpp"
#include "light_corner_corrector.hpp"
#include "number_classifier.hpp"
#include "auto_aim/debug/visualize/detect_debug.hpp"

namespace auto_aim
{
class ArmorDetector
{
  public:
      ArmorDetector(const std::string &config_path = "config/auto_aim.yaml");
      /**
       * @brief 识别装甲板
       * @param bgr_img 输入图像
       * @return 识别到的装甲板
       */
      std::vector<Armor> ArmorDetect(const cv::Mat &bgr_img) noexcept;

      void ArmorDetector_Set_Color(EnemyColor color)
      {
          detect_color_ = color;
      }

      void disable_debug() noexcept { detect_debug_.set_enabled(false); } 

  private:
    /**
     * @brief 寻找灯条
     * @param bin_img 二值化图像
     * @param src_img 输入图像
     * @return std::vector<LightBar>，找到的灯条
     */
    std::vector<LightBar> findLights(const cv::Mat &bin_img, const cv::Mat &src_img) noexcept;

    /**
     * @brief 按目标颜色筛选灯条，保留中性灯条供后续配板
     * @param lightbar 待筛选灯条，判色后更新颜色及目标颜色确认标志
     * @param src_img 原始 BGR 图像；为空或非三通道时跳过判色，保留原字段
     * @return true 保留灯条，不代表已确认目标颜色
     * @return false 灯条颜色相反或采样轴退化
     */
    bool filterLightColor(LightBar &lightbar, const cv::Mat &src_img) const;

    /**
     * @brief 根据灯条匹配装甲板
     * @param lights 找到的灯条
     * @return std::vector<Armor>，匹配到的装甲板
     */
    std::vector<Armor> findArmors(const std::vector<LightBar> &lights, const cv::Mat &src_img) noexcept;

    /**
     * @brief 检查灯条是否存在共用灯条的情况
     * @param i 灯条1
     * @param j 灯条2
     * @param lights 所有灯条
     * @return true 灯条1和灯条2共用灯条
     * @return false 灯条1和灯条2不共用灯条
     */
    bool containLight(const int i, const int j, const std::vector<LightBar> &lights) noexcept;

    std::vector<LightBar> lights_;
    std::vector<Armor>    armors_;

    // 缓存图像，避免重复申请内存
    cv::Mat gray_;
    cv::Mat binary_;

    DetectDebug detect_debug_;

    // yaml参数
    int binary_thres_;

    EnemyColor detect_color_  = EnemyColor::RED;

    double     min_lightbar_ratio_;
    double     max_lightbar_ratio_;
    double     min_lightbar_length_;
    double     max_angle_error_;
    double     max_lightbar_area_;
    double     min_armor_ratio_;
    double     max_armor_ratio_;
    double     max_side_ratio_;
    double     max_rectangular_error_;

    // 数字识别器
    std::unique_ptr<NumberClassifier> classifier;

    // 灯条角点优化器
    LightCornerCorrector light_corner_corrector_;
};

} // namespace auto_aim

#endif // _ARMOR_DETECTOR_H_
