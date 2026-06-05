/**
 * @file force_profile_generator.hpp
 * @brief 外骨骼关节力廓线生成器 — 基于步态相位的参考力矩曲线
 *
 * 将步态周期 (0%~100%) 映射为膝关节和踝关节的参考力矩输出。
 * 将力廓线参数化，支持分段控制策略:
 *
 * 膝关节 (KneeForceProfileGenerator):
 *   - 支撑相初期 (3%~30%): 虚拟刚度控制  τ = -K · θ_knee
 *   - 摆动相前期 (~42%~69%): Hermite 插值平滑弯曲力矩峰
 *   - 摆动相后期 (80%~98%):  虚拟阻尼控制  τ = -B · ω_knee
 *
 * 踝关节 (AnkleForceProfileGenerator):
 *   - 蹬地期 (28%~67%): Hermite 插值平滑跖屈力矩峰
 *   - 其余相位: 输出为零
 *
 * 每条轮廓由 5 个节点定义: {起点, 上升沿起点, 峰值点, 下降沿终点, 终点}
 * 使用分段三次 Hermite 插值在节点间平滑过渡。
 */
#ifndef FORCE_PROFILE_GENERATOR_HPP
#define FORCE_PROFILE_GENERATOR_HPP

#include <cstdint>
#include "piecewise_Hermite_interp.hpp"

/**
 * @brief 膝关节力廓线生成器
 *
 * 三段式力矩控制策略:
 *   1. 刚性段 (支撑相):  阻抗控制，输出与膝角成正比的伸膝力矩
 *   2. 插值段 (摆动前期): Hermite 插值曲线，产生助力屈膝力矩峰
 *   3. 阻尼段 (摆动后期): 粘滞阻尼，抑制膝关节过冲
 */
class KneeForceProfileGenerator
{
public:
    KneeForceProfileGenerator();
    ~KneeForceProfileGenerator() = default;

    /**
     * @brief 根据步态相位和膝关节状态计算参考力矩
     * @param gait_phase_percent  步态相位 (0~100%)
     * @param knee_angle_rad      膝关节角度 (rad)，正值表示屈曲
     * @param knee_velocity       膝关节角速度 (rad/s)
     * @return 参考力矩 (Nm/kg)，正值表示屈膝方向
     */
    float GetForceProfile(float gait_phase_percent, float knee_angle_rad, float knee_velocity);

    float stiffness_onset_phase_percent_;  /*!< 刚性段起始相位 (%) */
    float stiffness_offset_phase_percent_; /*!< 刚性段结束相位 (%) */
    float stiffness_;                      /*!< 虚拟刚度系数 (Nm/kg/rad) */

    HermiteInterp force_profile_interp_;   /*!< 插值段 Hermite 插值器 */
    float peak_time_phase_percent_;        /*!< 力矩峰值所在相位 (%) */
    float peak_torque_Nmkg_;               /*!< 峰值力矩幅值 (Nm/kg) */
    float rise_time_phase_percent_;        /*!< 上升段持续时间 (% 相位跨度) */
    float fall_time_phase_percent_;        /*!< 下降段持续时间 (% 相位跨度) */

    float damping_onset_phase_percent_;    /*!< 阻尼段起始相位 (%) */
    float damping_offset_phase_percent_;   /*!< 阻尼段结束相位 (%) */
    float damping_;                        /*!< 虚拟阻尼系数 (Nm/kg/(rad/s)) */
};

/**
 * @brief 踝关节力廓线生成器
 *
 * 单一 Hermite 插值曲线: 在蹬地期产生跖屈力矩峰。
 * 5 个节点: {0%, onset, peak, offset, 100%}，起点/终点值为 0。
 */
class AnkleForceProfileGenerator
{
public:
    AnkleForceProfileGenerator();
    ~AnkleForceProfileGenerator() = default;

    /**
     * @brief 根据步态相位计算踝关节参考力矩
     * @param gait_phase_percent  步态相位 (0~100%)
     * @return 参考力矩 (Nm/kg)，正值表示跖屈方向
     */
    float GetForceProfile(float gait_phase_percent);

    float start_time_phase_percent_;       /*!< 力矩起始相位 (%) */
    float end_time_phase_percent_;         /*!< 力矩结束相位 (%) */

    float peak_time_phase_percent_;        /*!< 力矩峰值所在相位 (%) */
    float peak_torque_Nmkg_;               /*!< 峰值力矩幅值 (Nm/kg) */
    float rise_time_phase_percent_;        /*!< 上升段相位跨度 (%) */
    float fall_time_phase_percent_;        /*!< 下降段相位跨度 (%) */
    HermiteInterp force_profile_interp_;   /*!< 插值段 Hermite 插值器 */
};


#endif
