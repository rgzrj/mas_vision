#include "rune_predictor.hpp"

namespace auto_buff {

void RunePredictor::reset()
{
    fitter_.reset();
    force_sine_until_ = rm_utils::Timestamp{};
}

void RunePredictor::correct(int inactive_corrected, int active_corrected, rm_utils::Timestamp current_stamp, double t_now,
                            double rotation_angle, RunePrediction &prediction)
{
    constexpr double kFitWarmupSeconds = 1.0;
    // 同帧匹配多个待击板时，沿用上游三秒正弦模型优先窗口。
    if (inactive_corrected > 1)
        force_sine_until_ = current_stamp + std::chrono::seconds{3};

    if (inactive_corrected > 0 || active_corrected > 0)
    {
        prediction.update_count += 1;

        // 跳过初始化后的首秒，再由拟合器自身检查最小时间跨度与样本窗口。
        if (t_now >= kFitWarmupSeconds)
        {
            fitter_.push(t_now, rotation_angle);

            auto res_linear = fitter_.fit_linear();
            auto res_sine   = fitter_.fit_sine();

            // 常规比较两种模型的代价，并限制正弦振幅；多待击板时优先窗口可覆盖该比较。
            if (res_sine && (!res_linear || current_stamp < force_sine_until_ ||
                             (res_sine->cost < res_linear->cost && res_sine->a >= 0.6)))
            {
                prediction.use_prediction_speed = false;
                prediction.sine_cost            = res_sine->cost;
                prediction.sine_C               = res_sine->C;
                prediction.sine_v               = res_sine->v;
                prediction.sine_a               = res_sine->a;
                prediction.sine_omega           = res_sine->omega;
                // 保存“本帧相位”，后续外推只增加 omega*dt，不再把绝对时间重复代入。
                prediction.sine_phase = res_sine->omega * t_now + res_sine->phi;
                prediction.sine_t     = t_now;
                prediction.sine_valid = true;
            }
            else if (res_linear)
            {
                prediction.prediction_speed     = res_linear->speed;
                prediction.use_prediction_speed = true;
                prediction.prediction_cost      = res_linear->cost;
                prediction.sine_valid           = false;
            }
        }
    }
}

} // namespace auto_buff
