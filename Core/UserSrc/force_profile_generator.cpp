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
    stiffness_onset_phase_percent_ = 3.0f;
    stiffness_offset_phase_percent_ = 30.0f;
    stiffness_ = 0.1f;

    peak_time_phase_percent_ = 56.0f;
    peak_torque_Nmkg_ = 0.05f;
    rise_time_phase_percent_ = 14.0f;
    fall_time_phase_percent_ = 13.0f;

    damping_onset_phase_percent_ = 80.0f;
    damping_offset_phase_percent_ = 98.0f;
    damping_ = 0.02f;

    float ptr_xs[5] = {stiffness_offset_phase_percent_, peak_time_phase_percent_ - rise_time_phase_percent_, peak_time_phase_percent_, peak_time_phase_percent_ + fall_time_phase_percent_, damping_onset_phase_percent_};
    float ptr_ys[5] = {0, 0, peak_torque_Nmkg_, 0, 0};
    float ptr_dys[5] = {0, 0, 0, 0, 0};
    uint16_t num_xs = 5;

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

        float stiffness_ramp_phase_percent = 10.0f;
        float stiffness_phase = gait_phase_percent - stiffness_onset_phase_percent_;
        float stiffness_gain = 1.0f;
        if (stiffness_phase < stiffness_ramp_phase_percent)
        {
            float t = stiffness_phase / stiffness_ramp_phase_percent;
            stiffness_gain = t * t * (3.0f - 2.0f * t); // smoothstep
        }
        stiffness_gain = 1.0f; /* HACK: 暂时不用平滑 */

        torque_profile = - stiffness_ * stiffness_gain * knee_angle_rad_limit;
    }
    else if (gait_phase_percent >= stiffness_offset_phase_percent_ && gait_phase_percent < damping_onset_phase_percent_)
    {
        // 从已经插值的力矩曲线中查表（线性插值）
        if (force_profile_interp_.ptr_y_interp_ == nullptr || force_profile_interp_.num_interp_ == 0)
        {
            torque_profile = 0.0f;
        }
        else
        {
            float x0 = force_profile_interp_.x_interp_start_;
            float dx = force_profile_interp_.x_interp_interval_;
            // 计算相对于插值起点的索引（浮点)
            float idxf = (gait_phase_percent - x0) / dx;
            // 限制范围
            if (idxf <= 0.0f)
            {
                torque_profile = force_profile_interp_.ptr_y_interp_[0];
            }
            else if (idxf >= (float)(force_profile_interp_.num_interp_ - 1))
            {
                torque_profile = force_profile_interp_.ptr_y_interp_[force_profile_interp_.num_interp_ - 1];
            }
            else
            {
                int i0 = (int)floorf(idxf);
                int i1 = i0 + 1;
                float y0 = force_profile_interp_.ptr_y_interp_[i0];
                float y1 = force_profile_interp_.ptr_y_interp_[i1];
                float frac = idxf - (float)i0;
                torque_profile = y0 + frac * (y1 - y0);
            }
        }
    }
    else if (gait_phase_percent >= damping_onset_phase_percent_ && gait_phase_percent < damping_offset_phase_percent_)
    {
        torque_profile =  - damping_ * knee_velocity;    }
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
    uint16_t num_xs = 5;

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
        // 从已经插值的力矩曲线中查表（线性插值）
        if (force_profile_interp_.ptr_y_interp_ == nullptr || force_profile_interp_.num_interp_ == 0)
        {
            torque_profile = 0.0f;
        }
        else
        {
            float x0 = force_profile_interp_.x_interp_start_;
            float dx = force_profile_interp_.x_interp_interval_;
            // 计算相对于插值起点的索引（浮点）
            float idxf = (gait_phase_percent - x0) / dx;
            // 限制范围
            if (idxf <= 0.0f)
            {
                torque_profile = force_profile_interp_.ptr_y_interp_[0];
            }
            else if (idxf >= (float)(force_profile_interp_.num_interp_ - 1))
            {
                torque_profile = force_profile_interp_.ptr_y_interp_[force_profile_interp_.num_interp_ - 1];
            }
            else
            {
                int i0 = (int)floorf(idxf);
                int i1 = i0 + 1;
                float y0 = force_profile_interp_.ptr_y_interp_[i0];
                float y1 = force_profile_interp_.ptr_y_interp_[i1];
                float frac = idxf - (float)i0;
                torque_profile = y0 + frac * (y1 - y0);
            }
        }
    }

    return torque_profile;
}
