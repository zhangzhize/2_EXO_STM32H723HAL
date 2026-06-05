/**
 * @file disturbance_observer.hpp
 * @brief 线性扩张状态观测器 (Linear Extended State Observer, LESO)
 *
 * 基于二阶系统模型 x1_ddot = f(x1, t) + b·u，将总扰动 f 扩展为状态 x2。
 * 观测器通过带宽参数 eso_w 配置极点，根据输入 u 和输出 x1 估计状态 x1_hat 和总扰动 x2_hat。
 * 常用于外骨骼关节力矩控制中的扰动补偿前馈。
 *
 * 状态方程:
 *   dot_hat_x1 = hat_x2 + 2·eso_w·(x1 - hat_x1) + u
 *   dot_hat_x2 = eso_w²·(x1 - hat_x1)
 *
 * 参考: Zhiqiang Gao, "Scaling and Bandwidth-Parameterization Based Controller Tuning"
 */
#ifndef DISTURBANCE_OBSERVER_HPP
#define DISTURBANCE_OBSERVER_HPP

#include <cstdint>

class DisturbanceObserver
{
public:
    /**
     * @brief 构造扰动观测器
     * @param eso_w 观测器带宽 (rad/s)，决定观测器收敛速度
     */
    DisturbanceObserver(float eso_w);
    ~DisturbanceObserver() = default;

    /**
     * @brief 执行一次观测器更新
     * @param x1 系统输出（被控对象实测值）
     * @param u  系统输入（控制量）
     */
    void UpdateObserver(float x1, float u);

    /** @brief 复位观测器状态为零 */
    void ResetObserver(void);

    float hat_x1_;        /*!< 状态 x1 的估计值（系统输出估计） */
    float hat_x2_;        /*!< 总扰动 f 的估计值（扩张状态） */
    float eso_w_;         /*!< 观测器带宽 (rad/s) */
    uint64_t tprev_sys_us_; /*!< 上次更新时的系统微秒时间戳 */
};

#endif