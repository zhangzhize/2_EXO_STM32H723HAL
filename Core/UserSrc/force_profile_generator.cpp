#include "force_profile_generator.hpp"
#include "utils.h"
extern "C" {
#include "arm_math.h"
}
/**
 * @brief Construct a new Knee Force Profile Generator:: Knee Force Profile Generator object
 * 
 */
KneeForceProfileGenerator::KneeForceProfileGenerator()
{
    /* 支撑相初期提出伸膝力矩, 力矩 = - 设定刚度 * 膝弯曲角度 */
    stiffness_onset_phase_percent_ = 3.0f;
    stiffness_offset_phase_percent_ = 30.0f;
    stiffness_ = 0.1f;

    /* 摆动相前期提供基于Piecewise Hermite插值的平滑弯曲力矩 */
    peak_time_phase_percent_ = 56.0f;
    peak_torque_Nmkg_ = 0.05f;
    rise_time_phase_percent_ = 14.0f;
    fall_time_phase_percent_ = 13.0f;

    /* 摆动相后期提供阻尼力矩, 力矩 = - 设定阻尼 * 膝角速度 */
    damping_onset_phase_percent_ = 80.0f;
    damping_offset_phase_percent_ = 98.0f;
    damping_ = 0.02f;

    float ptr_xs[5] = {stiffness_offset_phase_percent_, peak_time_phase_percent_ - rise_time_phase_percent_, peak_time_phase_percent_, peak_time_phase_percent_ + fall_time_phase_percent_, damping_onset_phase_percent_};
    float ptr_ys[5] = {0, 0, peak_torque_Nmkg_, 0, 0};
    float ptr_dys[5] = {0, 0, 0, 0, 0};
    uint16_t num_xs = sizeof(ptr_xs) / sizeof(float);

    force_profile_interp_ = HermiteInterp();
    force_profile_interp_.CalCoeffs(ptr_xs, ptr_ys, ptr_dys, num_xs);
    force_profile_interp_.Interp(0.1);
}

float KneeForceProfileGenerator::GetForceProfile(float gait_phase_percent, float knee_angle_rad, float knee_velocity)
{
    float torque_profile = 0.0f;
    if (gait_phase_percent < 0.0f || gait_phase_percent > 100.0f)
    {
        torque_profile = 0.0f;
    }
    else if (gait_phase_percent >= 0 && gait_phase_percent < stiffness_onset_phase_percent_)
    {
        torque_profile = 0.0f;
    }
    else if (gait_phase_percent >= stiffness_onset_phase_percent_ && gait_phase_percent < stiffness_offset_phase_percent_)
    {
        float knee_angle_rad_limit = knee_angle_rad;
        if (knee_angle_rad_limit < 0.0f) knee_angle_rad_limit = 0.0f;

        float stiffness_gain = 1.0f;

        #if 0 /* 暂时不用平滑刚度 */
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
    else if (gait_phase_percent >= stiffness_offset_phase_percent_ && gait_phase_percent < damping_onset_phase_percent_)
    {
        torque_profile = force_profile_interp_.Sample(gait_phase_percent);
    }
    else if (gait_phase_percent >= damping_onset_phase_percent_ && gait_phase_percent < damping_offset_phase_percent_)
    {
        torque_profile =  - damping_ * knee_velocity;
    }
    else
    {
        torque_profile = 0.0f;
    }

    return torque_profile;
}

AnkleForceProfileGenerator::AnkleForceProfileGenerator()
{
    start_time_phase_percent_ = 28.0f;
    end_time_phase_percent_ = 67.0f;

    peak_time_phase_percent_ = 54.0f;
    peak_torque_Nmkg_ = 0.7f;
    rise_time_phase_percent_ = peak_time_phase_percent_ - start_time_phase_percent_;
    fall_time_phase_percent_ = end_time_phase_percent_ - peak_time_phase_percent_;

    float ptr_xs[5] = {0, start_time_phase_percent_, peak_time_phase_percent_, end_time_phase_percent_, 100.0f};
    float ptr_ys[5] = {0, 0, peak_torque_Nmkg_, 0, 0};
    float ptr_dys[5] = {0, 0, 0, 0, 0};
    uint16_t num_xs = sizeof(ptr_xs) / sizeof(float);

    force_profile_interp_ = HermiteInterp();
    force_profile_interp_.CalCoeffs(ptr_xs, ptr_ys, ptr_dys, num_xs);
    force_profile_interp_.Interp(0.1);
}

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
