#include "adaptive_oscillator.hpp"
#include "utils.h"
extern "C" {
#include "arm_math.h"
}

#include "vofa.hpp"
#include "main.h"

AdaptiveOscillator::AdaptiveOscillator() :
    v_phi_(10.0f), v_omega_(10.0f), eta_(1.0f), kp_(1.0f), rho_(0.7f),
    gait_event_detect_mode_(GaitEventDetectMode::kPressureSensor)
{
    ResetGaitPhase();
}

void AdaptiveOscillator::UpdateGaitPhase(float left_angle_rad, float right_angle_rad, float left_velocity, float right_velocity, bool left_heel_contact_state, bool right_heel_contact_state)
{
    /**< 计算时间戳 */
    uint64_t tnow_sys_us = GetSysTimeUs();
    float tnow_sys_s = tnow_sys_us * 1e-6f;
    uint64_t delta_ts_us = tnow_sys_us - tprev_sys_us_;
    float delta_ts_s = 0;
    if (delta_ts_us > 0.01 * 1000000)
    {
        delta_ts_us = 0.01 * 1000000;
        delta_ts_s = 0.01;
    }
    else
    {
        delta_ts_s = (tnow_sys_us - tprev_sys_us_) * 1e-6f;
    }

    /**< 示教信号选择角度差 */
    float x_teach = left_angle_rad - right_angle_rad;

    /**< 检测步态事件 */
    bool is_left_angle_pos_peak = false;
    bool is_right_angle_pos_peak = false;
    static float left_angle_prev = 0.0f;
    static float left_angle_pprev = 0.0f;
    static float right_angle_prev = 0.0f;
    static float right_angle_pprev = 0.0f;
    static float left_angle_pos_peak_prev = 0.45f;
    static float right_angle_pos_peak_prev = 0.45f;
    float left_angle_pos_peak_threshold = 0.0f;
    float right_angle_pos_peak_threshold = 0.0f;

    bool is_left_heel_strike_event = false;
    bool is_right_heel_strike_event = false;

    static bool left_heel_contact_state_prev = false;
    static bool right_heel_contact_state_prev = false;

    if (gait_event_detect_mode_ == GaitEventDetectMode::kKneeAngleDiff)
    {
        left_angle_pos_peak_threshold = fmaxf(0.45f, 0.7f * left_angle_pos_peak_prev);
        right_angle_pos_peak_threshold = fmaxf(0.45f, 0.7f * right_angle_pos_peak_prev);

        if ((left_angle_rad < 2) && (left_angle_rad > left_angle_pos_peak_threshold) && (left_angle_rad - left_angle_prev <= 0) && (left_angle_prev - left_angle_pprev >= 0))
        {
            is_left_angle_pos_peak = true;
            left_angle_pos_peak_prev = left_angle_rad;
        }
        else if ((right_angle_rad < 2) && (right_angle_rad > right_angle_pos_peak_threshold) && (right_angle_rad - right_angle_prev <= 0) && (right_angle_prev - right_angle_pprev >= 0))
        {
            is_right_angle_pos_peak = true;
            right_angle_pos_peak_prev = right_angle_rad;
        }
    }
    else if (gait_event_detect_mode_ == GaitEventDetectMode::kPressureSensor)
    {
        if (left_heel_contact_state && !left_heel_contact_state_prev)
        {
            is_left_heel_strike_event = true;
        }
        if (right_heel_contact_state && !right_heel_contact_state_prev)
        {
            is_right_heel_strike_event = true;
        }
    }
    left_angle_pprev = left_angle_prev;
    left_angle_prev = left_angle_rad;
    right_angle_pprev = right_angle_prev;
    right_angle_prev = right_angle_rad;
    left_heel_contact_state_prev = left_heel_contact_state;
    right_heel_contact_state_prev = right_heel_contact_state;

    static uint64_t low_energy_duration_us = 0;
    float vel_energy = sqrtf(left_velocity*left_velocity + right_velocity*right_velocity);
    float alpha = 1.0f;
    float vel_energy_thresh = 0.0f;
    if (gait_event_detect_mode_ == GaitEventDetectMode::kKneeAngleDiff)
    {
        if (delta_ts_s > 0.0f)
        {
            alpha = 1.0f - expf(- delta_ts_s / KEmaTauS);
        }
        if ((left_event_cnt_ >= 1) != (right_event_cnt_ >= 1))
        {
            ema_vel_rms_ = vel_energy;
        }
        ema_vel_rms_ = alpha * vel_energy + (1.0f - alpha) * ema_vel_rms_;
        vel_energy_thresh = fmaxf(0.5f, 0.6f * ema_vel_rms_);
        if (vel_energy < vel_energy_thresh)
        {
            low_energy_duration_us += delta_ts_us;
        }
        else
        {
            low_energy_duration_us = 0;
        }
    }
    bool is_stopping_low_energy = (low_energy_duration_us > kMaxStoppingDurationUs);

    static uint64_t both_heel_contact_duration_us = 0;
    if (gait_event_detect_mode_ == GaitEventDetectMode::kPressureSensor)
    {
        if (left_heel_contact_state && right_heel_contact_state)
        {
            both_heel_contact_duration_us += delta_ts_us;
        }
        else
        {
            both_heel_contact_duration_us = 0;
        }
    }
    bool is_stopping_both_heel_contact = (both_heel_contact_duration_us > kMaxStoppingDurationUs);
    // bool is_stopping = is_stopping_low_energy || is_stopping_both_heel_contact;
    bool is_stopping = is_stopping_both_heel_contact;

    bool is_long_time_no_event = ((tnow_sys_us - left_tk_sys_us_ > kMaxTstrideUs) && (left_event_cnt_ >= 1)) || ((tnow_sys_us - right_tk_sys_us_ > kMaxTstrideUs) && (right_event_cnt_ >= 1));

    bool is_two_side_event_cnt_abnormal = (left_event_cnt_ > right_event_cnt_ + 1) || (right_event_cnt_ > left_event_cnt_ + 1);


    // gptr_vofa->ptr_vofa_data_[0] = is_long_time_no_event;
    // gptr_vofa->ptr_vofa_data_[1] = is_two_side_event_cnt_abnormal;
    // gptr_vofa->ptr_vofa_data_[2] = is_stopping_low_energy;
    // gptr_vofa->ptr_vofa_data_[3] = is_stopping_both_heel_contact;
    // gptr_vofa->ptr_vofa_data_[4] = left_heel_contact_state;
    // gptr_vofa->ptr_vofa_data_[5] = right_heel_contact_state;
    // gptr_vofa->SendOneFrame(6);


    if (is_long_time_no_event || is_two_side_event_cnt_abnormal || is_stopping)
    {
        left_event_cnt_ = 0;
        right_event_cnt_ = 0;
        left_angle_pos_peak_prev = 0.45f;
        right_angle_pos_peak_prev = 0.45f;
    }

    bool is_left_event = false;
    bool is_right_event = false;
    if (gait_event_detect_mode_ == GaitEventDetectMode::kKneeAngleDiff)
    {
        is_left_event = is_left_angle_pos_peak;
        is_right_event = is_right_angle_pos_peak;
    }
    else if (gait_event_detect_mode_ == GaitEventDetectMode::kPressureSensor)
    {
        is_left_event = is_left_heel_strike_event;
        is_right_event = is_right_heel_strike_event;
    }

    if (is_left_event)
    {
        if ((left_event_cnt_ < 1) || ((left_event_cnt_ >= 1) && (tnow_sys_us - left_tk_sys_us_ > kMinTstrideUs)))
        {
            left_event_cnt_ ++;
            left_tk_sys_us_ = tnow_sys_us;
            is_left_event = true;
        }
    }
    if (is_right_event)
    {
        if ((right_event_cnt_ < 1) || ((right_event_cnt_ >= 1) && (tnow_sys_us - right_tk_sys_us_ > kMinTstrideUs)))
        {
            right_event_cnt_ ++;
            right_tk_sys_us_ = tnow_sys_us;
            is_right_event = true;
        }
    }

    /**< 更新振荡器 */
    float dot_phi[kNumAOs] = {0};
    float dot_omega = 0;
    float dot_alpha[kNumAOs] = {0};
    float dot_alpha0 = 0;
    if (left_event_cnt_ >= 1 || right_event_cnt_ >= 1)
    {
        hat_x_ = alpha0_;
        float A = fabsf(alpha0_);
        for (uint8_t i=0; i<kNumAOs; i++)
        {
            hat_x_ += alpha_[i] * arm_sin_f32(phi_[i]);
            A += fabsf(alpha_[i]);
        }
        float e = x_teach - hat_x_;
        float e_norm = e / A;

        float sin_val, cos_val;
        for (uint8_t i=0; i<kNumAOs; i++)
        {
            arm_sin_cos_f32(RAD_TO_DEG * phi_[i], &sin_val, &cos_val);
            dot_phi[i] = omega_ * (i + 1) + v_phi_ * e_norm * cos_val;
            dot_alpha[i] = eta_ * e * sin_val;
            phi_[i] += dot_phi[i] * delta_ts_s;
            alpha_[i] += dot_alpha[i] * delta_ts_s;
        }
        dot_omega = v_omega_ * e_norm * arm_cos_f32(phi_[0]);
        dot_alpha0 = eta_ * e;
        omega_ += dot_omega * delta_ts_s;
        alpha0_ += dot_alpha0 * delta_ts_s;
    }
    else
    {
        omega_ = _2PI * 1.0;
        arm_fill_f32(0.0f, phi_, kNumAOs);
        arm_fill_f32(0.2f, alpha_, kNumAOs);
        alpha0_ = 0.0f;
    }
    
    float phi_n = fmodf(phi_[0], _2PI);
    if (phi_n < 0.0f) phi_n += _2PI;

    float Pe = 0;
    if ((phi_n >= 0.0f) && (phi_n < _PI))
    {
        Pe = - phi_n;
    }
    else if ((phi_n >= _PI) && (phi_n < _2PI))
    {
        Pe = _2PI - phi_n;
    }

    if (is_left_event)
    {
        if ((Pe > _PI/2.0f) && (left_Pe_tilde_tk_ < -_PI/2.0f))
        {
            left_Pe_tilde_tk_ = Pe - _2PI;
        }
        else if ((Pe < -_PI/2.0f) && (left_Pe_tilde_tk_ > _PI/2.0f))
        {
            left_Pe_tilde_tk_ = Pe + _2PI;
        }
        else
        {
            left_Pe_tilde_tk_ = Pe;
        }
        left_epsilon_phi_tk_ = kp_ * (left_Pe_tilde_tk_ - left_phi_e_);
    }
    float left_tk_s = left_tk_sys_us_ * 1e-6f;
    float dot_left_phi_e = left_epsilon_phi_tk_ * omega_ * expf(-omega_ * (tnow_sys_s - left_tk_s));
    left_phi_e_ += dot_left_phi_e * delta_ts_s;

    left_phi_comp_rad_ = fmodf(phi_n + left_phi_e_, _2PI);
    if (left_phi_comp_rad_ < 0.0f) left_phi_comp_rad_ += _2PI;

    if (is_right_event)
    {
        if ((Pe > _PI/2.0f) && (right_Pe_tilde_tk_ < -_PI/2.0f))
        {
            right_Pe_tilde_tk_ = Pe - _2PI;
        }
        else if ((Pe < -_PI/2.0f) && (right_Pe_tilde_tk_ > _PI/2.0f))
        {
            right_Pe_tilde_tk_ = Pe + _2PI;
        }
        else
        {
            right_Pe_tilde_tk_ = Pe;
        }
        right_epsilon_phi_tk_ = kp_ * (right_Pe_tilde_tk_ - right_phi_e_);
    }
    float right_tk_s = right_tk_sys_us_ * 1e-6f;
    float dot_right_phi_e = right_epsilon_phi_tk_ * omega_ * expf(-omega_ * (tnow_sys_s - right_tk_s));
    right_phi_e_ += dot_right_phi_e * delta_ts_s;
    right_phi_comp_rad_ = fmodf(phi_n + right_phi_e_, _2PI);
    if (right_phi_comp_rad_ < 0.0f) right_phi_comp_rad_ += _2PI;

    tprev_sys_us_ = tnow_sys_us;
}

void AdaptiveOscillator::ResetGaitPhase(void)
{
    tprev_sys_us_ = GetSysTimeUs();

    hat_x_ = 0;
    omega_ = _2PI * 1.0;
    arm_fill_f32(0.0f, phi_, kNumAOs);
    arm_fill_f32(0.2f, alpha_, kNumAOs);
    alpha0_ = 0.0f;

    left_phi_comp_rad_ = 0.0f;
    right_phi_comp_rad_ = 0.0f;

    left_tk_sys_us_ = GetSysTimeUs();
    right_tk_sys_us_ = GetSysTimeUs();
    left_Pe_tilde_tk_ = 0.0f;
    right_Pe_tilde_tk_ = 0.0f;
    left_epsilon_phi_tk_ = 0.0f;
    right_epsilon_phi_tk_ = 0.0f;
    left_phi_e_ = 0.0f;
    right_phi_e_ = 0.0f;

    left_event_cnt_ = 0;
    right_event_cnt_ = 0;
    ema_vel_rms_ = 0;
}
