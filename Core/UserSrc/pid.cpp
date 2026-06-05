#include "pid.hpp"
#include "utils.h"

float PIDController::operator()(float error)
{
    /* 计算距上次调用的时间间隔 */
    unsigned long timestamp_now = GetSysTimeUs();
    float ts = (timestamp_now - timestamp_prev_) * 1e-6f;
    /* 异常处理: 微秒计数器溢出回绕或首次调用时 ts 异常，强制为合理默认值 1ms */
    if (ts <= 0 || ts > 0.5f) ts = 1e-3f;

    /* u(s) = (P + I/s + Ds)·e(s)，以下为离散化实现 */

    /* 比例项: u_p(k) = Kp · e(k) */
    float proportional_out = kp_ * error;

    /* 积分项 (Tustin 双线性变换):
     *   u_i(k) = u_i(k-1) + Ki·Ts/2 · (e(k) + e(k-1)) */
    float integral_out = integral_prev_ + ki_ * ts * 0.5f * (error + error_prev_);
    /* 抗积分饱和 — 对积分分量钳位，防止在输出饱和时积分项无限累积 */
    integral_out = _constrain(integral_out, -output_limit_, output_limit_);

    /* 微分项 (后向差分):
     *   u_d(k) = Kd · (e(k) - e(k-1)) / Ts */
    float derivative_out = kd_ * (error - error_prev_) / ts;

    /* 三项求和 */
    float output = proportional_out + integral_out + derivative_out;
    /* 抗积分饱和 — 对总输出钳位 */
    output = _constrain(output, -output_limit_, output_limit_);

    /* 输出变化率斜坡限制: 防止输出阶跃对执行器造成冲击 */
    if (output_ramp_ > 0)
    {
        float output_rate = (output - output_prev_) / ts;
        if (output_rate > output_ramp_)
            output = output_prev_ + output_ramp_ * ts;
        else if (output_rate < -output_ramp_)
            output = output_prev_ - output_ramp_ * ts;
    }

    /* 保存状态供下次迭代使用 */
    integral_prev_ = integral_out;
    output_prev_ = output;
    error_prev_ = error;
    timestamp_prev_ = timestamp_now;
    return output;
}

void PIDController::ResetError(void)
{
    integral_prev_ = 0.0f;
    output_prev_ = 0.0f;
    error_prev_ = 0.0f;
    timestamp_prev_ = GetSysTimeUs();
}
