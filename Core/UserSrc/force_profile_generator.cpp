#include "force_profile_generator.hpp"
#include "utils.h"
extern "C" {
#include "arm_math.h"
}

/**
 * @brief 构造膝关节力廓线生成器 — 使用默认参数初始化三段式力矩轮廓
 */
KneeForceProfileGenerator::KneeForceProfileGenerator()
{
    /* 刚性段 (支撑相初期): 虚拟刚度产生的伸膝力矩 τ = -K·θ_knee
       仅当膝盖弯曲时输出力矩 (θ > 0 → τ < 0, 即伸膝方向) */
    stiffness_onset_phase_percent_ = 3.0f;
    stiffness_offset_phase_percent_ = 30.0f;
    stiffness_ = 0.1f;

    /* 摆动相前期: Hermite 插值产生平滑屈膝力矩峰
       峰值在 56% 步态相位，上升 14%，下降 13% */
    peak_time_phase_percent_ = 56.0f;
    peak_torque_Nmkg_ = 0.05f;
    rise_time_phase_percent_ = 14.0f;
    fall_time_phase_percent_ = 13.0f;

    /* 阻尼段 (摆动相后期): 虚拟阻尼抑制膝关节过冲 τ = -B·ω_knee */
    damping_onset_phase_percent_ = 80.0f;
    damping_offset_phase_percent_ = 98.0f;
    damping_ = 0.02f;

    /* 构建 5 节点 Hermite 插值轮廓:
       节点 x: {刚性段结束, 上升沿起点, 峰值点, 下降沿终点, 阻尼段开始}
       节点 y: {0, 0, peak, 0, 0} — 两端为零，中间一个峰值
       导数 dy: 全部为 0 — 保证节点处平滑过渡 */
    float ptr_xs[5] = {stiffness_offset_phase_percent_, peak_time_phase_percent_ - rise_time_phase_percent_, peak_time_phase_percent_, peak_time_phase_percent_ + fall_time_phase_percent_, damping_onset_phase_percent_};
    float ptr_ys[5] = {0, 0, peak_torque_Nmkg_, 0, 0};
    float ptr_dys[5] = {0, 0, 0, 0, 0};
    uint16_t num_xs = sizeof(ptr_xs) / sizeof(float);

    force_profile_interp_ = HermiteInterp();
    force_profile_interp_.CalCoeffs(ptr_xs, ptr_ys, ptr_dys, num_xs);
    force_profile_interp_.Interp(0.1);  /* 预计算 0.1% 步长插值表 */
}

/**
 * @brief 计算膝关节参考力矩
 *
 * 根据步态相位分段选择控制策略:
 *   - 越界 (< 0 或 > 100): 零输出
 *   - 刚性段: τ = -K·θ  (θ 钳位 ≥ 0, 避免后伸产生屈膝力矩)
 *   - 插值段: τ = force_profile_interp_.Sample(phase)
 *   - 阻尼段: τ = -B·ω
 *   - 其余: 零输出
 *
 * @param gait_phase_percent  步态相位 (0~100)
 * @param knee_angle_rad      膝关节角度 (rad, 正值=屈曲)
 * @param knee_velocity       膝关节角速度 (rad/s, 正值=屈曲方向)
 * @return 参考力矩 (Nm/kg，正值=屈膝方向)
 */
float KneeForceProfileGenerator::GetForceProfile(float gait_phase_percent, float knee_angle_rad, float knee_velocity)
{
    float torque_profile = 0.0f;

    /* 相位越界保护 */
    if (gait_phase_percent < 0.0f || gait_phase_percent > 100.0f)
    {
        torque_profile = 0.0f;
    }
    /* 0% ~ 刚性段起始: 零输出 */
    else if (gait_phase_percent >= 0 && gait_phase_percent < stiffness_onset_phase_percent_)
    {
        torque_profile = 0.0f;
    }
    /* 刚性段: τ = -K·θ, θ 钳位非负 (仅伸膝方向有力矩) */
    else if (gait_phase_percent >= stiffness_onset_phase_percent_ && gait_phase_percent < stiffness_offset_phase_percent_)
    {
        float knee_angle_rad_limit = knee_angle_rad;
        if (knee_angle_rad_limit < 0.0f) knee_angle_rad_limit = 0.0f;

        float stiffness_gain = 1.0f;

        #if 0 /* 预留: 平滑刚度斜坡 (S 曲线过渡)，暂未启用 */
        float stiffness_ramp_phase_percent = 10.0f;
        float stiffness_phase = gait_phase_percent - stiffness_onset_phase_percent_;
        if (stiffness_phase < stiffness_ramp_phase_percent)
        {
            float t = stiffness_phase / stiffness_ramp_phase_percent;
            stiffness_gain = t * t * (3.0f - 2.0f * t);
        }
        #endif

        torque_profile = - stiffness_ * stiffness_gain * knee_angle_rad_limit;
    }
    /* 插值段: Hermite 插值力矩峰 */
    else if (gait_phase_percent >= stiffness_offset_phase_percent_ && gait_phase_percent < damping_onset_phase_percent_)
    {
        torque_profile = force_profile_interp_.Sample(gait_phase_percent);
    }
    /* 阻尼段: τ = -B·ω */
    else if (gait_phase_percent >= damping_onset_phase_percent_ && gait_phase_percent < damping_offset_phase_percent_)
    {
        torque_profile =  - damping_ * knee_velocity;
    }
    /* 98%~100%: 零输出 */
    else
    {
        torque_profile = 0.0f;
    }

    return torque_profile;
}

/**
 * @brief 构造踝关节力廓线生成器 — 使用默认参数初始化蹬地期力矩峰
 */
AnkleForceProfileGenerator::AnkleForceProfileGenerator()
{
    start_time_phase_percent_ = 28.0f;
    end_time_phase_percent_ = 67.0f;

    peak_time_phase_percent_ = 54.0f;
    peak_torque_Nmkg_ = 0.7f;
    rise_time_phase_percent_ = peak_time_phase_percent_ - start_time_phase_percent_;
    fall_time_phase_percent_ = end_time_phase_percent_ - peak_time_phase_percent_;

    /* 构建 5 节点 Hermite 插值轮廓:
       节点 x: {0, 开始, 峰值, 结束, 100}
       节点 y: {0, 0, peak, 0, 0}
       导数 dy: 全部为 0 */
    float ptr_xs[5] = {0, start_time_phase_percent_, peak_time_phase_percent_, end_time_phase_percent_, 100.0f};
    float ptr_ys[5] = {0, 0, peak_torque_Nmkg_, 0, 0};
    float ptr_dys[5] = {0, 0, 0, 0, 0};
    uint16_t num_xs = sizeof(ptr_xs) / sizeof(float);

    force_profile_interp_ = HermiteInterp();
    force_profile_interp_.CalCoeffs(ptr_xs, ptr_ys, ptr_dys, num_xs);
    force_profile_interp_.Interp(0.1);  /* 预计算 0.1% 步长插值表 */
}

/**
 * @brief 计算踝关节参考力矩
 *
 * 踝关节仅有一段 Hermite 插值力矩峰 (蹬地期跖屈助力)，
 * 其余相位输出为零。
 *
 * @param gait_phase_percent  步态相位 (0~100)
 * @return 参考力矩 (Nm/kg，正值=跖屈方向)
 */
float AnkleForceProfileGenerator::GetForceProfile(float gait_phase_percent)
{
    float torque_profile = 0.0f;
    if (gait_phase_percent < 0.0f || gait_phase_percent > 100.0f)
    {
        torque_profile = 0.0f;
    }
    else
    {
        torque_profile = force_profile_interp_.Sample(gait_phase_percent);
    }

    return torque_profile;
}
