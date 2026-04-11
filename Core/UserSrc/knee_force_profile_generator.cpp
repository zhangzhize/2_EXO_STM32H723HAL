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
    stiffness_onset_phase_rad_ = 0.03 * _2PI;
    stiffness_offset_phase_rad_ = 0.30 * _2PI;
    stiffness_ = 0.1f;

    peak_time_phase_rad_ = 0.56 * _2PI;
    peak_torque_Nmkg_ = 0.15f;
    rise_time_phase_rad_ = 0.14 * _2PI;
    fall_time_phase_rad_ = 0.13 * _2PI;

    damping_onset_phase_rad_ = 0.80 * _2PI;
    damping_offset_phase_rad_ = 0.98 * _2PI;
    damping_ = 0.02f;

    float ptr_xs[5] = {stiffness_offset_phase_rad_, peak_time_phase_rad_ - rise_time_phase_rad_, peak_time_phase_rad_, peak_time_phase_rad_ + fall_time_phase_rad_, damping_onset_phase_rad_};
    float ptr_ys[5] = {0, 0, peak_torque_Nmkg_, 0, 0};
    float ptr_dys[5] = {0, 0, 0, 0, 0};
    uint16_t num_xs = 5;

    force_profile_interp_ = HermiteInterp();
    force_profile_interp_.CalCoeffs(ptr_xs, ptr_ys, ptr_dys, num_xs);
    force_profile_interp_.Interp(0.01);
}

float KneeForceProfileGenerator::GetForceProfile(float gait_phase_rad, float knee_angle_rad, float knee_velocity)
{
    float torque_profile = 0.0f;
    if (gait_phase_rad < 0.0f || gait_phase_rad > _2PI)
    {
        torque_profile = 0.0f;
    }
    else if (gait_phase_rad >= 0 && gait_phase_rad < stiffness_onset_phase_rad_)
    {
        torque_profile = 0.0f;
    }
    else if (gait_phase_rad >= stiffness_onset_phase_rad_ && gait_phase_rad < stiffness_offset_phase_rad_)
    {
        float knee_angle_rad_limit = knee_angle_rad;
        if (knee_angle_rad_limit < 0.0f) knee_angle_rad_limit = 0.0f;
        
        torque_profile = - stiffness_ * knee_angle_rad_limit;
        // torque_profile = 0;
    }
    else if (gait_phase_rad >= stiffness_offset_phase_rad_ && gait_phase_rad < damping_onset_phase_rad_)
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
            float idxf = (gait_phase_rad - x0) / dx;
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
    else if (gait_phase_rad >= damping_onset_phase_rad_ && gait_phase_rad < damping_offset_phase_rad_)
    {
        // torque_profile =  - damping_ * knee_velocity;
        torque_profile = 0;
    }
    else
    {
        torque_profile = 0.0f;
    }

    return torque_profile;
}

AnkleForceProfileGenerator::AnkleForceProfileGenerator()
{
    start_time_phase_rad_ = 0.28 * _2PI;
    end_time_phase_rad_ = 0.67 * _2PI;

    peak_time_phase_rad_ = 0.54 * _2PI;
    peak_torque_Nmkg_ = 0.7f;
    rise_time_phase_rad_ = peak_time_phase_rad_ - start_time_phase_rad_;
    fall_time_phase_rad_ = end_time_phase_rad_ - peak_time_phase_rad_;

    float ptr_xs[5] = {0, start_time_phase_rad_, peak_time_phase_rad_, end_time_phase_rad_, _2PI};
    float ptr_ys[5] = {0, 0, peak_torque_Nmkg_, 0, 0};
    float ptr_dys[5] = {0, 0, 0, 0, 0};
    uint16_t num_xs = 5;

    force_profile_interp_ = HermiteInterp();
    force_profile_interp_.CalCoeffs(ptr_xs, ptr_ys, ptr_dys, num_xs);
    force_profile_interp_.Interp(0.01);
}

float AnkleForceProfileGenerator::GetForceProfile(float gait_phase_rad)
{
    float torque_profile = 0.0f;
    if (gait_phase_rad < 0.0f || gait_phase_rad > _2PI)
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
            float idxf = (gait_phase_rad - x0) / dx;
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
