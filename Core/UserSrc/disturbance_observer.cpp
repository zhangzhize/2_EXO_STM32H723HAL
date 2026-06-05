#include "disturbance_observer.hpp"
#include "utils.h"


DisturbanceObserver::DisturbanceObserver(float eso_w)
    : hat_x1_(0.0f), hat_x2_(0.0f), eso_w_(eso_w), tprev_sys_us_(GetSysTimeUs())
{
}

/**
 * @brief 更新观测器状态
 * @param x1 被控对象输出（实测值）
 * @param u  控制输入
 *
 * 状态方程 (连续域):
 *   dot_hat_x1 = hat_x2 + 2·eso_w·(x1 - hat_x1) + u
 *   dot_hat_x2 = eso_w²·(x1 - hat_x1)
 *
 * 离散化采用前向欧拉法: x(k+1) = x(k) + dot_x(k) · ts
 * ts > 10ms 时强制钳位为 10ms，防止长时间未调用导致的数值发散。
 */
void DisturbanceObserver::UpdateObserver(float x1, float u)
{
    float ts = 0.0f;
    uint64_t tnow_sys_us = GetSysTimeUs();
    /* 若距上次更新超过 1s (如初始化后首次调用)，使用默认 10ms 步长避免 ts 过大 */
    if (tnow_sys_us - tprev_sys_us_ > 1000000)
    {
        ts = 0.01f;
        tprev_sys_us_ = tnow_sys_us;
    }
    else
    {
        ts = (tnow_sys_us - tprev_sys_us_) * 0.000001;
        tprev_sys_us_ = tnow_sys_us;
    }

    /* 连续域状态导数计算 */
    float dot_hat_x1 = hat_x2_ + 2 * eso_w_ * (x1 - hat_x1_) + u;
    float dot_hat_x2 = eso_w_ * eso_w_ * (x1 - hat_x1_);
    /* 前向欧拉离散化: x(k+1) = x(k) + dot_x(k)·Ts */
    hat_x1_ = hat_x1_ + dot_hat_x1 * ts;
    hat_x2_ = hat_x2_ + dot_hat_x2 * ts;
}

/**
 * @brief 复位观测器状态
 *
 * 将 hat_x1 和 hat_x2 归零，通常在系统重新标定或控制模式切换时调用。
 */
void DisturbanceObserver::ResetObserver(void)
{
    hat_x1_ = 0.0f;
    hat_x2_ = 0.0f;
}