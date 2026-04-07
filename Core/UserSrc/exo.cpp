#include "exo.hpp"
#include "utils.h"
#include <string.h>
#include <algorithm>
#include <cstring>
#include <cstdio>
extern "C" {
#include "arm_math.h"
}

extern uint32_t g_adc_data[3];  /**< definition in alt_main.cpp */


enum ExoJointCanID : uint8_t
{
    kLeftHip = 0x01,
    kRightHip = 0x02,
    kLeftKnee = 0x2A,
    kRightKnee = 0x55,
    kLeftAnkle = 0x2A,
    kRightAnkle = 0x55,
};

ImuData::ImuData(bool is_left)
{
    is_left_ = is_left;
    is_used_ = true;

    quat_i_ = 0.0f;
    quat_j_ = 0.0f;
    quat_k_ = 0.0f;
    quat_real_ = 1.0f;
}

JointData::JointData(bool is_left)
{
    is_left_ = is_left;
    is_used_ = true;

    rom_rad_ = 0.0f;
    pos_rad_ = 0.0f;
    vel_radps_ = 0.0f;
    tor_Nm_ = 0.0f;
}

SideData::SideData(bool is_left) : hip_joint_(is_left), knee_joint_(is_left), ankle_joint_(is_left), foot_imu_(is_left)
{
    is_left_ = is_left;
    is_used_ = true;

    /**< from Side */
    prev_heel_contact_state_ = true;
    prev_toe_contact_state_ = true;

    for (int i = 0; i<kNumStepsAvg; i++)
    {
        step_times_[i] = 0;
        stance_times_[i] = 0;
        swing_times_[i] = 0;
    }

    ground_strike_timestamp_ = 0;
    prev_ground_strike_timestamp_ = 0;
    toe_strike_timestamp_ = 0;
    prev_toe_strike_timestamp_ = 0;
    toe_off_timestamp_ = 0;
    prev_toe_off_timestamp_ = 0;

    /**< from SideData */
    percent_gait_ = -1.0f; 
    percent_stance_ = -1.0f;
    percent_swing_ = -1.0f;
    expected_step_duration_ = -1.0f; 
    expected_swing_duration_ = -1.0f;
    expected_stance_duration_ = -1.0f;

    ground_strike_ = false; 
    toe_off_ = false;
    toe_strike_ = false;
    toe_on_ = false;
    heel_contact_state_ = false;
    toe_contact_state_ = false;

    is_calibration_done_ = false;
    do_calibration_toe_fsr_ = true;
    do_calibration_refinement_toe_fsr_ = true;
    do_calibration_heel_fsr_ = true;
    do_calibration_refinement_heel_fsr_ = true;

    expected_duration_window_upper_coeff_ = 1.75f;
    expected_duration_window_lower_coeff_ = 0.25f;
}

ExoData::ExoData() : left_side_(true), right_side_(false)
{
    user_weight_kg_ = 60.0f;
    battery_voltage_ = 24.0f;
    enable_vofa_telemetry_ = false;

    state_ = State::kSleep;
    error_code_ = Error::kNone;
    pending_events_ = SysEvent::kNone;
    loco_mode_ = LocoMode::kWalking;
    override_usr_.enable_locomode_override = false;
    override_usr_.forced_locomode = LocoMode::kWalking;

    ao_left_phase_rad_ = 0.0f;
    ao_right_phase_rad_ = 0.0f;
    ao_left_event_cnt_ = 0;
    ao_right_event_cnt_ = 0;
}

AnkleJoint::AnkleJoint(bool is_left, ExoData *pe) : pe_(pe), ps_(is_left ? &pe->left_side_ : &pe->right_side_), pj_(is_left ? &pe->left_side_.ankle_joint_ : &pe->right_side_.ankle_joint_), force_profile_generator_(), motor_(is_left ? ExoJointCanID::kLeftAnkle : ExoJointCanID::kRightAnkle), pid_(0.2f, 0.4f, 0.0f, -1.0f, 10.0f)
{
    cable_pre_tensioned_position_ = 0.0f;
    cable_tensioned_position_ = 6.0f;
    cable_released_position_ = 0.0f;
    assistance_start_phase_percent_ = 35.0f;
    assistance_end_phase_percent_ = 65.0f;
}

void AnkleJoint::Calibrate()
{
    if (!pj_->is_used_)
    {
        return;
    }
    /** #Todo: Implement joint calibration logic, if needed */
}

bool AnkleJoint::IsMotorConnect()
{
    if (!pj_->is_used_)
    {
        return true;
    }

    if (motor_.feedback_flag_ > 0)
    {
        return true;
    }
    motor_.run_mode_ = RobstrideMotorMode::kMotionMode;
    motor_.SetMotorMode();
    // motor_.EnableMotor();
    return false;
}

void AnkleJoint::Shutdown()
{
    motor_.DisableMotor(0);
}

void AnkleJoint::Standby()
{
    if (!pj_->is_used_)
    {
        return;
    }

    // motor_.torque_forward_ = 0.0f;
    // motor_.position_ref_ = 0.1f;
    // motor_.speed_ref_ = 0.0f;
    // motor_.motion_mode_kp_ = 0.0f;
    // motor_.motion_mode_kd_ = 0.0f;
    // motor_.MotionControl();
    motor_.EnableMotor();
}

void AnkleJoint::Read()
{
    if (!pj_->is_used_)
    {
        return;
    }

    if (pj_->is_left_)
    {
        pj_->pos_rad_ = motor_.position_;
        pj_->vel_radps_ = motor_.speed_;
        pj_->tor_Nm_ = motor_.torque_;
    }
    else
    {
        pj_->pos_rad_ = -motor_.position_;
        pj_->vel_radps_ = -motor_.speed_;
        pj_->tor_Nm_ = -motor_.torque_;
    }
}

void AnkleJoint::Assist()
{
    if (!pj_->is_used_)
    {
        return;
    }

    /** 1. 获取 FSR 算出的相位百分比 (0.0f~100.0f) */
    float phase_percent = pj_->is_left_ ? pe_->left_side_.percent_gait_ : pe_->right_side_.percent_gait_;

    /** 2. 根据步态相位设置参考位置 */
    float cable_position_ref = cable_released_position_;
    if (phase_percent >= 0.0f && phase_percent < 100.0f) 
    {
        if (phase_percent < assistance_start_phase_percent_) 
        {
            cable_position_ref = cable_pre_tensioned_position_;
        }
        else if (phase_percent >= assistance_start_phase_percent_ && phase_percent < assistance_end_phase_percent_) 
        {
            cable_position_ref = cable_tensioned_position_;
        }
        else 
        {
            cable_position_ref = cable_released_position_;
        }
    }

    if (!pj_->is_left_)
    {
        cable_position_ref = -cable_position_ref;
    }

    /** 3. 发送电机控制指令 */
    motor_.position_ref_ = cable_position_ref;
    motor_.torque_forward_ = 0.0f;
    motor_.speed_ref_ = 0.0f;
    motor_.motion_mode_kp_ = 15.0f;
    motor_.motion_mode_kd_ = 0.5f;
    motor_.MotionControl();
}

KneeJoint::KneeJoint(bool is_left, ExoData *pe) : pe_(pe), ps_(is_left ? &pe->left_side_ : &pe->right_side_), pj_(is_left ? &pe->left_side_.knee_joint_ : &pe->right_side_.knee_joint_), force_profile_generator_(), motor_(is_left ? ExoJointCanID::kLeftKnee : ExoJointCanID::kRightKnee), disturbance_observer_(5.0f)
{
}

void KneeJoint::Calibrate()
{
    if (!pj_->is_used_)
    {
        return;
    }
    /** #Todo: Implement joint calibration logic, if needed  */
}

bool KneeJoint::IsMotorConnect()
{
    if (!pj_->is_used_)
    {
        return true;
    }

    if (motor_.feedback_flag_ > 0)
    {
        return true;
    }
    motor_.run_mode_ = RobstrideMotorMode::kMotionMode;
    motor_.SetMotorMode();
    // motor_.EnableMotor();
    return false;
}

void KneeJoint::Shutdown()
{
    motor_.DisableMotor(0);
}

void KneeJoint::Standby()
{
    if (!pj_->is_used_)
    {
        return;
    }

    // motor_.torque_forward_ = 0.0f;
    // motor_.position_ref_ = 0.0f;
    // motor_.speed_ref_ = 0.0f;
    // motor_.motion_mode_kp_ = 0.0f;
    // motor_.motion_mode_kd_ = 0.0f;
    // motor_.MotionControl();
    motor_.EnableMotor();
}

void KneeJoint::Read()
{
    if (!pj_->is_used_)
    {
        return;
    }

    if (pj_->is_left_)
    {
        pj_->pos_rad_ = motor_.position_;
        pj_->vel_radps_ = motor_.speed_;
        pj_->tor_Nm_ = motor_.torque_;
    }
    else
    {
        pj_->pos_rad_ = -motor_.position_;
        pj_->vel_radps_ = -motor_.speed_;
        pj_->tor_Nm_ = -motor_.torque_;
    }
}

void KneeJoint::Assist()
{
    if (!pj_->is_used_)
    {
        return;
    }

    float force_profile = 0.0f;

    // float phase_rad = pj_->is_left_ ? pe_->ao_left_phase_rad_ : pe_->ao_right_phase_rad_;
    float phase_rad = pj_->is_left_ ? pe_->left_side_.percent_gait_ : pe_->right_side_.percent_gait_;
    phase_rad = phase_rad * _2PI / 100.0f;
    // uint32_t gait_event_cnt = pj_->is_left_ ? pe_->ao_left_event_cnt_ : pe_->ao_right_event_cnt_;

    // float phase_rad = pj_->is_left_ ? pe_->ao_left_phase_rad_ : pe_->ao_right_phase_rad_;
    // float phase_percent = phase_rad * _2PI / 100.0f;

    // float phase_percent = pj_->is_left_ ? pe_->left_side_.percent_gait_ : pe_->right_side_.percent_gait_;

    // uint32_t gait_event_cnt = pj_->is_left_ ? pe_->ao_left_event_cnt_ : pe_->ao_right_event_cnt_;

    force_profile = force_profile_generator_.GetForceProfile(phase_rad, pj_->pos_rad_, pj_->vel_radps_);
    if (!pj_->is_left_)
    {
        force_profile = -force_profile;
    }
    motor_.torque_forward_ = force_profile * pe_->user_weight_kg_;
    motor_.position_ref_ = 0.0f;
    motor_.speed_ref_ = 0.0f;
    motor_.motion_mode_kp_ = 0.0f;
    motor_.motion_mode_kd_ = 0.0f;
    motor_.MotionControl();
}


void KneeJoint::ImpedanceControl()
{
    static float q_ref_ = 0.0f;
    static float dot_q_ref_ = 0.0f;
    static float ddot_q_ref_ = 0.0f;
    static uint64_t t0 = GetSysTimeUs();
    static float M0_ = 1.0f;
    static float Md_ = 0.0f;
    static float Bd_ = 1.0f;
    static float Kd_ = 1.0f;

    if (!pj_->is_used_)
    {
        return;
    }
    float omega = _2PI * 1.0f;
    float amplitude = 2.0f;
    float tnow_sys_us = GetSysTimeUs();
    float sin_t, cos_t;
    arm_sin_cos_f32(RAD_TO_DEG * omega * (tnow_sys_us - t0) / 1000000.0f, &sin_t, &cos_t);
    q_ref_ = amplitude * sin_t;
    dot_q_ref_ = amplitude * omega * cos_t;
    ddot_q_ref_ = - amplitude * omega * omega * sin_t;
    disturbance_observer_.UpdateObserver(pj_->pos_rad_, pj_->tor_Nm_ / M0_);

    float hat_disturbance = - M0_ * disturbance_observer_.hat_x2_;
    float dot_tilde_q = dot_q_ref_ - pj_->vel_radps_;
    float tilde_q = q_ref_ - pj_->pos_rad_;

    motor_.torque_forward_ = M0_ * ddot_q_ref_ + M0_ / Md_ * (Bd_ * dot_tilde_q + Kd_ * tilde_q - hat_disturbance) + hat_disturbance;
    motor_.position_ref_ = 0;
    motor_.speed_ref_ = 0;
    motor_.motion_mode_kp_ = 0;
    motor_.motion_mode_kd_ = 0;
    motor_.MotionControl();
}

HipJoint::HipJoint(bool is_left, ExoData *exo_data) : pe_(exo_data), ps_(is_left ? &pe_->left_side_ : &pe_->right_side_), pj_(is_left ? &pe_->left_side_.hip_joint_ : &pe_->right_side_.hip_joint_), motor_(is_left ? ExoJointCanID::kLeftHip : ExoJointCanID::kRightHip)
{
}

void HipJoint::Calibrate()
{
    if (!pj_->is_used_)
    {
        return;
    }
    /** #Todo: Implement joint calibration logic, if needed  */
}

void HipJoint::Read()
{
    if (!pj_->is_used_)
    {
        return;
    }

    if (pj_->is_left_)
    {
        pj_->pos_rad_ = motor_.feedback_.pos_rad_;
        pj_->vel_radps_ = motor_.feedback_.vel_radps_;
        pj_->tor_Nm_ = motor_.feedback_.tor_Nm_;
    }
    else
    {
        pj_->pos_rad_ = -motor_.feedback_.pos_rad_;
        pj_->vel_radps_ = -motor_.feedback_.vel_radps_;
        pj_->tor_Nm_ = -motor_.feedback_.tor_Nm_;
    }
}

bool HipJoint::IsMotorConnect()
{
    if (!pj_->is_used_)
    {
        return true;
    }

    if (motor_.feedback_.flag_ > 0)
    {
        return true;
    }
    // motor_.ctrl_param_.mode_id_ = DMMotorModeID::kMIT;
    // motor_.SetMotorMode();
    motor_.EnableMotor();
    return false;
}

void HipJoint::Shutdown()
{
    motor_.DisableMotor();
}

void HipJoint::Standby()
{
    if (!pj_->is_used_)
    {
        return;
    }

    motor_.ctrl_param_.mode_id_ = DMMotorModeID::kMIT;
    motor_.ctrl_param_.kp_set_ = 0.0f;
    motor_.ctrl_param_.kd_set_ = 0.0f;
    motor_.ctrl_param_.pos_set_rad_ = 0.0f;
    motor_.ctrl_param_.vel_set_radps_ = 0.0f;
    motor_.ctrl_param_.tor_set_Nm_ = 0.0f;
    motor_.MitControl();
}

namespace {
struct HipFFShared
{
    static constexpr int kHistSize = 128;
    static constexpr int kDelaySamples = 35;     // delay 35*5ms
    static constexpr float kLambda = 0.01f;
    static constexpr float kK = 7.0f;

    float posL[kHistSize] = {0.0f};
    float posR[kHistSize] = {0.0f};
    float lastL = 0.0f;
    float lastR = 0.0f;
    int idx = 0;
    bool inited = false;

    uint32_t last_tick_ms = 0;

    float tau_left = 0.0f;
    float tau_right = 0.0f;

    void UpdateAndCompute(uint32_t now_ms, float posL_now, float posR_now)
    {
        if (last_tick_ms == now_ms) return;
        last_tick_ms = now_ms;

        if (!inited)
        {
            lastL = posL_now;
            lastR = posR_now;
            for (int i = 0; i < kHistSize; ++i)
            {
                posL[i] = posL_now;
                posR[i] = posR_now;
            }
            inited = true;
        }

        posL[idx] = kLambda * posL_now + (1.0f - kLambda) * lastL;
        posR[idx] = kLambda * posR_now + (1.0f - kLambda) * lastR;
        lastL = posL[idx];
        lastR = posR[idx];

        int past = idx - kDelaySamples;
        if (past < 0) past += kHistSize;

        const float posL_d = posL[past];
        const float posR_d = posR[past];

        const float tau = kK * (sinf(posR_d) - sinf(posL_d));
        tau_left = tau;
        // tau_right = -tau;
        tau_right = tau;

        idx = (idx + 1) % kHistSize;
    }
};
static HipFFShared g_hip_ff;
}

void HipJoint::Assist()
{
    if (!pj_->is_used_)
    {
        return;
    }

    const float posL_now = pe_->left_side_.hip_joint_.pos_rad_;
    const float posR_now = pe_->right_side_.hip_joint_.pos_rad_;
    const uint32_t now_ms = GetSysTimeMs();

    if (pj_->is_left_)
    {
        g_hip_ff.UpdateAndCompute(now_ms, posL_now, posR_now);
    }
    const float tau_me = pj_->is_left_ ? g_hip_ff.tau_left : g_hip_ff.tau_right;

    motor_.ctrl_param_.mode_id_ = DMMotorModeID::kMIT;
    motor_.ctrl_param_.kp_set_ = 0.0f;
    motor_.ctrl_param_.kd_set_ = 0.0f;
    motor_.ctrl_param_.pos_set_rad_ = 0.0f;
    motor_.ctrl_param_.vel_set_radps_ = 0.0f;
    motor_.ctrl_param_.tor_set_Nm_ = tau_me;
    motor_.MitControl();
    // motor_.EnableMotor();
}

Side::Side(bool is_left, ExoData *pe) : pe_(pe), ps_(is_left ? &pe_->left_side_ : &pe_->right_side_), heel_fsr_(is_left), toe_fsr_(is_left), hip_joint_(is_left, pe), knee_joint_(is_left, pe), ankle_joint_(is_left, pe)
{
}

void Side::Calibrate()
{
    hip_joint_.Calibrate();
    knee_joint_.Calibrate();
    ankle_joint_.Calibrate();
    CalibrateFsr();

    ps_->is_calibration_done_ = !(ps_->do_calibration_heel_fsr_ || ps_->do_calibration_refinement_heel_fsr_ || ps_->do_calibration_toe_fsr_ || ps_->do_calibration_refinement_toe_fsr_);
}

void Side::Read()
{
    if (!ps_->is_used_)
    {
        return;
    }

    hip_joint_.Read();
    knee_joint_.Read();
    ankle_joint_.Read();
    heel_fsr_.Read();
    toe_fsr_.Read();
}

bool Side::IsMotorConnect()
{
    if (!ps_->is_used_)
    {
        return true;
    }

    bool hip_ok = hip_joint_.IsMotorConnect();
    bool knee_ok = knee_joint_.IsMotorConnect();
    bool ankle_ok = ankle_joint_.IsMotorConnect();

    return hip_ok && knee_ok && ankle_ok;
}

void Side::Standby()
{
    if (!ps_->is_used_)
    {
        return;
    }

    hip_joint_.Standby();
    knee_joint_.Standby();
    ankle_joint_.Standby();
}

void Side::Assist()
{
    if (!ps_->is_used_)
    {
        return;
    }

    hip_joint_.Assist();
    knee_joint_.Assist();
    ankle_joint_.Assist();
}

void Side::CalibrateFsr()
{
    if (!ps_->is_used_)
    {
        return;
    }

    if (ps_->do_calibration_toe_fsr_)
    {
        ps_->do_calibration_toe_fsr_ = toe_fsr_.Calibrate(ps_->do_calibration_toe_fsr_);
    }
    else if (ps_->do_calibration_refinement_toe_fsr_)
    {
        ps_->do_calibration_refinement_toe_fsr_ = toe_fsr_.RefineCalibration(ps_->do_calibration_refinement_toe_fsr_);
    }

    if (ps_->do_calibration_heel_fsr_)
    {
        ps_->do_calibration_heel_fsr_ = heel_fsr_.Calibrate(ps_->do_calibration_heel_fsr_);
    }
    else if (ps_->do_calibration_refinement_heel_fsr_)
    {
        ps_->do_calibration_refinement_heel_fsr_ = heel_fsr_.RefineCalibration(ps_->do_calibration_refinement_heel_fsr_);
    }
}


void Side::FsrTimeBaseGaitPhaseEstimate()
{
    if (!ps_->is_used_)
    {
        return;
    }

    ps_->heel_contact_state_ = heel_fsr_.GetGroundContact();
    ps_->toe_contact_state_ = toe_fsr_.GetGroundContact();

    /**< Now use heel fsr only */
    // ps_->ground_strike_ = (!ps_->prev_heel_contact_state_ && !ps_->prev_toe_contact_state_) && ((ps_->heel_contact_state_ && !ps_->prev_heel_contact_state_) || (ps_->toe_contact_state_ && !ps_->prev_toe_contact_state_));
    ps_->ground_strike_ = (ps_->heel_contact_state_ && !ps_->prev_heel_contact_state_);
    ps_->toe_off_ = (!ps_->toe_contact_state_ && ps_->prev_toe_contact_state_);
    ps_->toe_strike_ = (ps_->toe_contact_state_ && !ps_->prev_toe_contact_state_);

    ps_->prev_heel_contact_state_ = ps_->heel_contact_state_;
    ps_->prev_toe_contact_state_ = ps_->toe_contact_state_;

    uint32_t now = GetSysTimeMs();
    if (ps_->ground_strike_)
    {
        ps_->prev_ground_strike_timestamp_ = ps_->ground_strike_timestamp_;
        ps_->ground_strike_timestamp_ = now;
        ps_->expected_step_duration_ = UpdateExpectedDuration();
    }
    if (ps_->toe_strike_)
    {
        ps_->prev_toe_strike_timestamp_ = ps_->toe_strike_timestamp_;
        ps_->toe_strike_timestamp_ = now;
        ps_->expected_swing_duration_ = UpdateExpectedSwingDuration();
    }
    if (ps_->toe_off_)
    {
        ps_->prev_toe_off_timestamp_ = ps_->toe_off_timestamp_;
        ps_->toe_off_timestamp_ = now;
        ps_->expected_stance_duration_ = UpdateExpectedStanceDuration();
    }

    if (ps_->expected_step_duration_ > 0.0f)
    {
        ps_->percent_gait_ = 100.0f * (((float)now - (float)ps_->ground_strike_timestamp_) / ps_->expected_step_duration_);
        if (ps_->percent_gait_ > 100.0f) ps_->percent_gait_ = 100.0f;
    }

    if (ps_->expected_stance_duration_ > 0.0f)
    {
        ps_->percent_stance_ = 100.0f * (((float)now - (float)ps_->toe_strike_timestamp_) / ps_->expected_stance_duration_);
        if (ps_->percent_stance_ > 100.0f) ps_->percent_stance_ = 100.0f;
    }
    if (!ps_->heel_contact_state_ && !ps_->toe_contact_state_)
    {
        ps_->percent_stance_ = 0.0f;
    }

    if (ps_->expected_swing_duration_ > 0.0f)
    {
        ps_->percent_swing_ = 100.0f * (((float)now - (float)ps_->toe_off_timestamp_) / ps_->expected_swing_duration_);
        if (ps_->percent_swing_ > 100.0f) ps_->percent_swing_ = 100.0f;
    }
    if (ps_->heel_contact_state_ || ps_->toe_contact_state_)
    {
        ps_->percent_swing_ = 0.0f;
    }
}

void Side::ClearStepTimeEstimate()
{
    for (int i = 0; i < ps_->kNumStepsAvg; i++)
    {
        ps_->step_times_[i] = 0;
        ps_->stance_times_[i] = 0;
        ps_->swing_times_[i] = 0;
    }
}

void Side::Shutdown()
{
    hip_joint_.motor_.DisableMotor();
    knee_joint_.motor_.DisableMotor(0);
    ankle_joint_.motor_.DisableMotor(0);
    ClearStepTimeEstimate();
}

float Side::UpdateExpectedDuration()
{
    uint32_t step_time = ps_->ground_strike_timestamp_ - ps_->prev_ground_strike_timestamp_;
    float expected_step_duration = ps_->expected_step_duration_;

    if (0 == ps_->prev_ground_strike_timestamp_)
    {
        return expected_step_duration;
    }

    uint8_t num_uninitialized = 0;
    for (int i = 0; i < ps_->kNumStepsAvg; i++)
    {
        num_uninitialized += (ps_->step_times_[i] == 0);
    }

    uint32_t* max_val = std::max_element(ps_->step_times_, ps_->step_times_ + ps_->kNumStepsAvg);
    uint32_t* min_val = std::min_element(ps_->step_times_, ps_->step_times_ + ps_->kNumStepsAvg);

    if  (num_uninitialized > 0)
    {
        for (int i = (ps_->kNumStepsAvg - 1); i>0; i--)
        {
            ps_->step_times_[i] = ps_->step_times_[i-1];
        }
        ps_->step_times_[0] = step_time;
    }

    else if ((step_time <= (ps_->expected_duration_window_upper_coeff_ * *max_val)) && (step_time >= (ps_->expected_duration_window_lower_coeff_ * *min_val)))
    {
        int sum_step_times = step_time;
        for (int i = (ps_->kNumStepsAvg - 1); i>0; i--)
        {
            sum_step_times += ps_->step_times_[i-1];
            ps_->step_times_[i] = ps_->step_times_[i-1];
        }
        ps_->step_times_[0] = step_time;

        expected_step_duration = sum_step_times / ps_->kNumStepsAvg;
    }
    return expected_step_duration;
}

float Side::UpdateExpectedStanceDuration()
{
    uint32_t stance_time = ps_->toe_off_timestamp_ - ps_->toe_strike_timestamp_;
    float expected_stance_duration = ps_->expected_stance_duration_;

    if (0 == ps_->prev_toe_strike_timestamp_)
    {
        return expected_stance_duration;
    }
    
    uint8_t num_uninitialized = 0;
    for (int i = 0; i < ps_->kNumStepsAvg; i++)
    {
        num_uninitialized += (ps_->stance_times_[i] == 0);
    }

    uint32_t* max_val = std::max_element(ps_->stance_times_, ps_->stance_times_ + ps_->kNumStepsAvg);
    uint32_t* min_val = std::min_element(ps_->stance_times_, ps_->stance_times_ + ps_->kNumStepsAvg);

    if (num_uninitialized > 0)
    {
        for (int i = (ps_->kNumStepsAvg - 1); i>0; i--)
        {
            ps_->stance_times_[i] = ps_->stance_times_[i - 1];
        }
        ps_->stance_times_[0] = stance_time;
    }
    else if ((stance_time <= (ps_->expected_duration_window_upper_coeff_ * *max_val)) && (stance_time >= (ps_->expected_duration_window_lower_coeff_ * *min_val)))
    {
        int sum_stance_times = stance_time;
        for (int i = (ps_->kNumStepsAvg - 1); i>0; i--)
        {
            sum_stance_times += ps_->stance_times_[i - 1];
            ps_->stance_times_[i] = ps_->stance_times_[i - 1];
        }
        ps_->stance_times_[0] = stance_time;

        expected_stance_duration = sum_stance_times / ps_->kNumStepsAvg;
    }
    return expected_stance_duration;
}

float Side::UpdateExpectedSwingDuration()
{
    uint32_t swing_time = ps_->toe_strike_timestamp_ - ps_->toe_off_timestamp_;
    float expected_swing_duration = ps_->expected_swing_duration_;

    if (0 == ps_->prev_toe_off_timestamp_)
    {
        return expected_swing_duration;
    }

    uint8_t num_uninitialized = 0;

    for (int i = 0; i < ps_->kNumStepsAvg; i++)
    {
        num_uninitialized += (ps_->swing_times_[i] == 0);
    }

    uint32_t* max_val = std::max_element(ps_->swing_times_, ps_->swing_times_ + ps_->kNumStepsAvg);
    uint32_t* min_val = std::min_element(ps_->swing_times_, ps_->swing_times_ + ps_->kNumStepsAvg);

    if (num_uninitialized > 0)
    {
        for (int i = (ps_->kNumStepsAvg - 1); i>0; i--)
        {
            ps_->swing_times_[i] = ps_->swing_times_[i - 1];
        }
        ps_->swing_times_[0] = swing_time;
    }
    else if ((swing_time <= (ps_->expected_duration_window_upper_coeff_ * *max_val)) && (swing_time >= (ps_->expected_duration_window_lower_coeff_ * *min_val)))
    {
        int sum_swing_times = swing_time;
        for (int i = (ps_->kNumStepsAvg - 1); i>0; i--)
        {
            sum_swing_times += ps_->swing_times_[i - 1];
            ps_->swing_times_[i] = ps_->swing_times_[i - 1];
        }
        ps_->swing_times_[0] = swing_time;

        expected_swing_duration = sum_swing_times / ps_->kNumStepsAvg;
    }
    return expected_swing_duration;
}

ExoShell::ExoShell(Exo* ptr_exo) : ptr_exo_(ptr_exo)
{
    RegisterCommand("setled", CmdWrapper<ExoShell, &ExoShell::OnCmdSetLed>, this);
    RegisterCommand("vofa", CmdWrapper<ExoShell, &ExoShell::OnCmdVofaTelemetry>, this);
    RegisterCommand("setlocomode", CmdWrapper<ExoShell, &ExoShell::OnCmdSetLocoMode>, this);

    RegisterCommand("estop", [](void* ctx, int, char**) {
        static_cast<ExoShell*>(ctx)->ptr_exo_->pe_->pending_events_ |= ExoData::SysEvent::kEmergencyStop;
        static_cast<ExoShell*>(ctx)->SendString("!!! EMERGENCY STOP !!!\r\n");
    }, this);
    RegisterCommand("wakeup", [](void* ctx, int, char**) {
        static_cast<ExoShell*>(ctx)->ptr_exo_->pe_->pending_events_ |= ExoData::SysEvent::kWakeup;
        static_cast<ExoShell*>(ctx)->SendString("-> Wakeup Requested\r\n");
    }, this);
    RegisterCommand("calib", [](void* ctx, int, char**) {
        static_cast<ExoShell*>(ctx)->ptr_exo_->pe_->pending_events_ |= ExoData::SysEvent::kStartCalibrate;
        static_cast<ExoShell*>(ctx)->SendString("-> Calibration Requested\r\n");
    }, this);
    RegisterCommand("start", [](void* ctx, int, char**) {
        static_cast<ExoShell*>(ctx)->ptr_exo_->pe_->pending_events_ |= ExoData::SysEvent::kStartAssist;
        static_cast<ExoShell*>(ctx)->SendString("-> Start Assist Requested\r\n");
    }, this);
    RegisterCommand("stop", [](void* ctx, int, char**) {
        static_cast<ExoShell*>(ctx)->ptr_exo_->pe_->pending_events_ |= ExoData::SysEvent::kStopAssist;
        static_cast<ExoShell*>(ctx)->SendString("-> Stop Assist Requested\r\n");
    }, this);
    RegisterCommand("sleep", [](void* ctx, int, char**) {
        static_cast<ExoShell*>(ctx)->ptr_exo_->pe_->pending_events_ |= ExoData::SysEvent::kEnterSleep;
        static_cast<ExoShell*>(ctx)->Printf("-> Sleep Requested\r\n");
    }, this);
    RegisterCommand("clearfaults", [](void* ctx, int, char**) {
        static_cast<ExoShell*>(ctx)->ptr_exo_->pe_->pending_events_ |= ExoData::SysEvent::kClearFaults;
        static_cast<ExoShell*>(ctx)->Printf("-> Clear Faults Requested\r\n");
    }, this);

    RegisterRwParam("weight", &ptr_exo_->pe_->user_weight_kg_);
    RegisterRwParam("la_on",  &ptr_exo_->left_side_.ankle_joint_.assistance_start_phase_percent_);
    RegisterRwParam("ra_on",  &ptr_exo_->right_side_.ankle_joint_.assistance_start_phase_percent_);
    RegisterRwParam("la_off", &ptr_exo_->left_side_.ankle_joint_.assistance_end_phase_percent_);
    RegisterRwParam("ra_off", &ptr_exo_->right_side_.ankle_joint_.assistance_end_phase_percent_);
}

void ExoShell::OnCmdSetLed(int argc, char** argv)
{
    if (argc < 2 || argv == nullptr || argv[1] == nullptr)
    {
        SendString("Usage: setled <number>\r\n");
        return;
    }

    int state = GetInt(argc, argv, 1);
    ptr_exo_->state_led_.UpdateColorBDMA(state);
    SendString("LED state set!\r\n");
}

void ExoShell::OnCmdVofaTelemetry(int argc, char** argv)
{
    if (argc < 2 || argv == nullptr || argv[1] == nullptr)
    {
        SendString("Usage: vofa on|off\r\n");
        return;
    }

    if (strcmp(argv[1], "on") == 0)
    {
        ptr_exo_->pe_->enable_vofa_telemetry_ = true;
    }
    else if (strcmp(argv[1], "off") == 0)
    {
        ptr_exo_->pe_->enable_vofa_telemetry_ = false;
        SendString("VOFA telemetry: OFF\r\n");
    }
    else
    {
        SendString("Usage: vofa on|off\r\n");
        return;
    }
}

void ExoShell::OnCmdSetLocoMode(int argc, char **argv)
{
    if (argc < 2 || argv == nullptr || argv[1] == nullptr)
    {
        SendString("Usage: setlocomode <mode: auto|walking|sit2stand...>\r\n");
        return;
    }

    const char *mode_str = GetString(argc, argv, 1, "");
    if (strcmp(mode_str, "auto") == 0)
    {
        ptr_exo_->pe_->override_usr_.enable_locomode_override = false;
        SendString("Mode Autonomous\r\n");
    }
    else if (strcmp(mode_str, "walking") == 0)
    {
        ptr_exo_->pe_->override_usr_.enable_locomode_override = true;
        ptr_exo_->pe_->override_usr_.forced_locomode = ExoData::LocoMode::kWalking;
        SendString("Mode set to: kWalking\r\n");
    }
    else if (strcmp(mode_str, "sit2stand") == 0)
    {
        ptr_exo_->pe_->override_usr_.enable_locomode_override = true;
        ptr_exo_->pe_->override_usr_.forced_locomode = ExoData::LocoMode::kSitToStand;
        SendString("Mode set to: kSitToStand\r\n");
    }
    else
    {
        Printf("Unknown mode: %s\r\n", mode_str);
    }
}

Exo::Exo(ExoData *pe) : pe_(pe), state_led_(), adaptive_oscilator_(), left_side_(true, pe), right_side_(false, pe), shell_(this)
{
}

void Exo::Initialize()
{
    /** 调试: 设置足底接触状态的施密特触发器阈值. */
    left_side_.heel_fsr_.lower_threshold_percent_ground_contact_ = 0.15f;
    left_side_.heel_fsr_.upper_threshold_percent_ground_contact_ = 0.25f;
    left_side_.toe_fsr_.lower_threshold_percent_ground_contact_ = 0.15f;
    left_side_.toe_fsr_.upper_threshold_percent_ground_contact_ = 0.25f;
    right_side_.heel_fsr_.lower_threshold_percent_ground_contact_ = 0.15f;
    right_side_.heel_fsr_.upper_threshold_percent_ground_contact_ = 0.25f;
    right_side_.toe_fsr_.lower_threshold_percent_ground_contact_ = 0.15f;
    right_side_.toe_fsr_.upper_threshold_percent_ground_contact_ = 0.25f; 

    /** 调试: 重置标定标志. */
    ResetCalibrationFlags();

    /** 调试: 选择助力的关节 */
    pe_->left_side_.hip_joint_.is_used_ = false;
    pe_->right_side_.hip_joint_.is_used_ = false;
    pe_->left_side_.knee_joint_.is_used_ = false;
    pe_->right_side_.knee_joint_.is_used_ = false;
    pe_->left_side_.ankle_joint_.is_used_ = true;
    pe_->right_side_.ankle_joint_.is_used_ = true;

    /** 调试: 髋关节参数. */

    /** 调试: 膝关节参数. */
    left_side_.knee_joint_.force_profile_generator_.stiffness_ = 0.5f;
    right_side_.knee_joint_.force_profile_generator_.stiffness_ = 0.5f;

    /** 调试: 踝关节参数 */
    left_side_.ankle_joint_.cable_released_position_ = 0.2f;
    left_side_.ankle_joint_.cable_pre_tensioned_position_ = 0.6f;
    left_side_.ankle_joint_.cable_tensioned_position_ = 1.4f;
    left_side_.ankle_joint_.assistance_start_phase_percent_ = 35.0f;
    left_side_.ankle_joint_.assistance_end_phase_percent_ = 65.0f;

    right_side_.ankle_joint_.cable_released_position_ = 0.2f;
    right_side_.ankle_joint_.cable_pre_tensioned_position_ = 0.4f;
    right_side_.ankle_joint_.cable_tensioned_position_ = 1.0f;
    right_side_.ankle_joint_.assistance_start_phase_percent_ = 35.0f;
    right_side_.ankle_joint_.assistance_end_phase_percent_ = 65.0f;
}

void Exo::Run()
{
    Read();
    if (pe_->state_ != ExoData::State::kSleep)
    {
        CheckSystemHealth();   /** 重新计算error_code_ */
    }

    /** 根据系统当前状态过滤无效事件 */
    pe_->pending_events_ &= AllowedEventsForState(pe_->state_);

    /** 紧急关停事件判断 */
    const bool is_estop_triggered = ((pe_->pending_events_ & ExoData::SysEvent::kEmergencyStop) != 0);
    if (is_estop_triggered)
    {
        ClearNonCriticalEvents(pe_);
        pe_->state_ = ExoData::State::kSleep;
        Shutdown();
        return;   /** 没清estop事件, 永久锁死 */
    }

    const bool battery_low = ((pe_->error_code_ & ExoData::Error::kBatteryLow) != 0);
    const bool has_any_fault = (pe_->error_code_ != ExoData::Error::kNone);
    if (battery_low)
    {
        if (pe_->state_ != ExoData::State::kFaultLowBattery)
        {
            ClearNonCriticalEvents(pe_);
            pe_->state_ = ExoData::State::kFaultLowBattery;
        }
    }
    else if (has_any_fault)
    {
        if (pe_->state_ != ExoData::State::kFaultSystem)
        {
            ClearNonCriticalEvents(pe_);
            pe_->state_ = ExoData::State::kFaultSystem;
        }
    }
    else if ((pe_->pending_events_ & ExoData::SysEvent::kEnterSleep) != 0)
    {
        pe_->pending_events_ &= ~ExoData::SysEvent::kEnterSleep;
        pe_->state_ = ExoData::State::kSleep;
        Shutdown();
    }

    switch (pe_->state_)
    {
    case ExoData::State::kSleep:
        if (((pe_->pending_events_ & ExoData::SysEvent::kWakeup) != 0) && pe_->battery_voltage_ >= 19.5f)
        {
            pe_->pending_events_ &= ~ExoData::SysEvent::kWakeup;
            pe_->state_ = ExoData::State::kWaitMotorComm;
        }
        break;

    case ExoData::State::kWaitMotorComm:
        if (IsMotorConnect())
        {
            /** 无需要用户命令启动 */
            // pe_->state_ = ExoData::State::kCalibrating;
            // ResetCalibrationFlags();
            /** 需要用户命令启动 */
            if ((pe_->pending_events_ & ExoData::SysEvent::kStartCalibrate) != 0)
            {
                pe_->pending_events_ &= ~ExoData::SysEvent::kStartCalibrate;
                ResetCalibrationFlags();
                pe_->state_ = ExoData::State::kCalibrating;
            }
        }
        break;

    case ExoData::State::kCalibrating:
        Calibrate();
        Standby();   /** 为了获取电机/关节状态, 保持通信 */
        if (IsCalibrateDone())
        {
            pe_->state_ = ExoData::State::kReady;
        }
        break;
    case ExoData::State::kReady:
        Estimate();
        Standby();   /** 为了获取电机/关节状态, 保持通信 */
        
        if ((pe_->pending_events_ & ExoData::SysEvent::kStartCalibrate) != 0)
        {
            pe_->pending_events_ &= ~ExoData::SysEvent::kStartCalibrate;
            ResetCalibrationFlags();
            pe_->state_ = ExoData::State::kCalibrating;
        }
        else if ((pe_->pending_events_ & ExoData::SysEvent::kStartAssist) != 0)
        {
            pe_->pending_events_ &= ~ExoData::SysEvent::kStartAssist;
            pe_->state_ = ExoData::State::kAssisting;
        }
        break;
    case ExoData::State::kAssisting:
        Estimate();
        if ((pe_->pending_events_ & ExoData::SysEvent::kStopAssist) != 0)
        {
            pe_->pending_events_ &= ~ExoData::SysEvent::kStopAssist;
            Standby();
            pe_->state_ = ExoData::State::kReady;
        }
        // else if (IsStaticMotionIntent())
        // {
        //     Standby();
        //     pe_->state_ = ExoData::State::kReady;
        // }
        else
        {
            Assist();
        }
        break;
    case ExoData::State::kFaultLowBattery:
        Shutdown();
        if (pe_->battery_voltage_ >= 19.5f)
        {
            pe_->state_ = ExoData::State::kSleep;
        }
        break;
    case ExoData::State::kFaultSystem:
        Shutdown();
        if (((pe_->pending_events_ & ExoData::SysEvent::kClearFaults) != 0))
        {
            pe_->pending_events_ &= ~ExoData::SysEvent::kClearFaults;
            pe_->error_code_ = ExoData::Error::kNone;
            pe_->state_ = ExoData::State::kSleep;
        }
        break;
    default:
        break;
    }

    if (shell_.ProcessPendingCommand())
    {
        pe_->vofa_pause_until_ms_ = GetSysTimeMs() + 3000U;
    }

    const uint32_t now_ms = GetSysTimeMs();
    if (pe_->enable_vofa_telemetry_ && (now_ms >= pe_->vofa_pause_until_ms_))
    {
        VofaSendTelemetry();
    }

    state_led_.UpdateColorBDMA(static_cast<uint8_t>(pe_->state_));
}

void Exo::Calibrate()
{
    left_side_.Calibrate();
    right_side_.Calibrate();
}

void Exo::ResetCalibrationFlags()
{
    pe_->left_side_.is_calibration_done_ = false;
    pe_->left_side_.do_calibration_heel_fsr_ = true;
    pe_->left_side_.do_calibration_toe_fsr_ = false;
    pe_->left_side_.do_calibration_refinement_heel_fsr_ = true;
    pe_->left_side_.do_calibration_refinement_toe_fsr_ = false;

    pe_->right_side_.is_calibration_done_ = false;
    pe_->right_side_.do_calibration_heel_fsr_ = true;
    pe_->right_side_.do_calibration_toe_fsr_ = false;
    pe_->right_side_.do_calibration_refinement_heel_fsr_ = true;
    pe_->right_side_.do_calibration_refinement_toe_fsr_ = false;

    /** 最终标定的结果是以下用于归一化的最值 */
    // left_side_.heel_fsr_.calibration_refinement_max_ = 2.45f;
    // left_side_.heel_fsr_.calibration_refinement_min_ = 0.15f;
    // right_side_.heel_fsr_.calibration_refinement_max_ = 2.45f;
    // right_side_.heel_fsr_.calibration_refinement_min_ = 0.15f;
}

void Exo::Read()
{
    pe_->battery_voltage_ = (g_adc_data[0] * 3.3f / 65535) * 11;
    // pe_->battery_voltage_ = 24; /** just for debug */
    left_side_.Read();
    right_side_.Read();
}

void Exo::Estimate()
{

    /** high level control */
    /** #TODO: 目前先由用户强制选择运动模式 */
    if (pe_->override_usr_.enable_locomode_override)
    {
        pe_->loco_mode_ = pe_->override_usr_.forced_locomode;
    }
    else
    {
        pe_->loco_mode_ = pe_->override_usr_.forced_locomode;
    }

    /** Mid level control */
    /** #TODO: 目前先调试平地步行辅助 */
    if (pe_->loco_mode_ == ExoData::LocoMode::kWalking)
    {
        /**  */
        left_side_.FsrTimeBaseGaitPhaseEstimate();
        right_side_.FsrTimeBaseGaitPhaseEstimate();

        adaptive_oscilator_.UpdateGaitPhase(pe_->left_side_.knee_joint_.pos_rad_, pe_->right_side_.knee_joint_.pos_rad_, pe_->left_side_.knee_joint_.vel_radps_, pe_->right_side_.knee_joint_.vel_radps_, pe_->left_side_.heel_contact_state_, pe_->right_side_.heel_contact_state_);
        pe_->ao_left_phase_rad_ = adaptive_oscilator_.left_phi_comp_rad_;
        pe_->ao_right_phase_rad_ = adaptive_oscilator_.right_phi_comp_rad_;
        pe_->ao_left_event_cnt_ = adaptive_oscilator_.left_event_cnt_;
        pe_->ao_right_event_cnt_ = adaptive_oscilator_.right_event_cnt_;
    }
}

void Exo::Standby()
{
    left_side_.Standby();
    right_side_.Standby();
}

void Exo::Assist()
{
    left_side_.Assist();
    right_side_.Assist();
}

void Exo::Shutdown()
{
    left_side_.Shutdown();
    right_side_.Shutdown();
}

void Exo::CheckSystemHealth()
{
    pe_->error_code_ = ExoData::Error::kNone;

    if (pe_->battery_voltage_ < 19.0f)
    {
        pe_->error_code_ |= ExoData::Error::kBatteryLow;
    }
    if (left_side_.knee_joint_.motor_.error_code_ != 0 || left_side_.knee_joint_.motor_.fault_code_ != 0)
    {
        pe_->error_code_ |= ExoData::Error::kLeftKneeFault;
    }
    if (right_side_.knee_joint_.motor_.error_code_ != 0 || right_side_.knee_joint_.motor_.fault_code_ != 0)
    {
        pe_->error_code_ |= ExoData::Error::kRightKneeFault;
    }
    if (left_side_.ankle_joint_.motor_.error_code_ != 0 || left_side_.ankle_joint_.motor_.fault_code_ != 0)
    {
        pe_->error_code_ |= ExoData::Error::kLeftAnkleFault;
    }
    if (right_side_.ankle_joint_.motor_.error_code_ != 0 || right_side_.ankle_joint_.motor_.fault_code_ != 0)
    {
        pe_->error_code_ |= ExoData::Error::kRightAnkleFault;
    }
}

/** #HACK: 发送想记录的数据 */
void Exo::VofaSendTelemetry()
{
    static uint32_t loop_cnt = 0;
    shell_.SetVofaJustFloatData(0, loop_cnt++);
    shell_.SetVofaJustFloatData(1, left_side_.heel_fsr_.raw_reading_);
    shell_.SetVofaJustFloatData(2, left_side_.toe_fsr_.raw_reading_);
    shell_.SetVofaJustFloatData(3, right_side_.heel_fsr_.raw_reading_);
    shell_.SetVofaJustFloatData(4, right_side_.toe_fsr_.raw_reading_);
    shell_.SetVofaJustFloatData(5, pe_->left_side_.percent_gait_ / 100.0f);
    shell_.SetVofaJustFloatData(6, pe_->right_side_.percent_gait_ / 100.0f);
    shell_.SetVofaJustFloatData(7, left_side_.ankle_joint_.motor_.position_ref_);
    shell_.SetVofaJustFloatData(8, right_side_.ankle_joint_.motor_.position_ref_);
    shell_.SetVofaJustFloatData(9, left_side_.ankle_joint_.motor_.position_);
    shell_.SetVofaJustFloatData(10, right_side_.ankle_joint_.motor_.position_);
    shell_.SendVofaJustFloatFrame(11);
}

bool Exo::IsMotorConnect()
{
    return left_side_.IsMotorConnect() && right_side_.IsMotorConnect();
}

bool Exo::IsCalibrateDone()
{
    return pe_->left_side_.is_calibration_done_ && pe_->right_side_.is_calibration_done_;
}

bool Exo::IsStopWalking()
{
    return pe_->ao_left_event_cnt_ <= 1 && pe_->ao_right_event_cnt_ <= 1;
}

ExoData::SysEvent Exo::AllowedEventsForState(ExoData::State state)
{
    ExoData::SysEvent allowed_events = ExoData::SysEvent::kEmergencyStop;
    switch (state)
    {
    case ExoData::State::kSleep:
        allowed_events |= ExoData::SysEvent::kWakeup;
        break;

    case ExoData::State::kWaitMotorComm:
        allowed_events |= ExoData::SysEvent::kStartCalibrate;
        allowed_events |= ExoData::SysEvent::kEnterSleep;
        break;
    
    case ExoData::State::kCalibrating:
        allowed_events |= ExoData::SysEvent::kEnterSleep;
        break;
        
    case ExoData::State::kReady:
        allowed_events |= ExoData::SysEvent::kStartCalibrate;
        allowed_events |= ExoData::SysEvent::kStartAssist;
        allowed_events |= ExoData::SysEvent::kEnterSleep;
        break;

    case ExoData::State::kAssisting:
        allowed_events |= ExoData::SysEvent::kStopAssist;
        allowed_events |= ExoData::SysEvent::kEnterSleep;
        break;

    case ExoData::State::kFaultSystem:
        allowed_events |= ExoData::SysEvent::kClearFaults;
        break;

    default:
        break;
    }
    return allowed_events;
}

void Exo::SensorUartRxCallback(uint8_t *data, uint16_t data_size)
{
    if (data_size == 56)
    {
        exo_sensor_packet_t *packet = (exo_sensor_packet_t *)data;
        left_side_.heel_fsr_.raw_reading_ = 3.4f - packet->left_foot.mV_heel / 1000.0f;
        left_side_.toe_fsr_.raw_reading_ = 3.4f - packet->left_foot.mV_toe / 1000.0f;
        left_side_.ps_->ankle_plantarflexion_force_N_ = packet->left_foot.mV_pull;
        left_side_.ps_->foot_imu_.quat_i_ = packet->left_foot.quatI;
        left_side_.ps_->foot_imu_.quat_j_ = packet->left_foot.quatJ;
        left_side_.ps_->foot_imu_.quat_k_ = packet->left_foot.quatK;
        left_side_.ps_->foot_imu_.quat_real_ = packet->left_foot.quatReal;
        
        right_side_.heel_fsr_.raw_reading_ = 3.4f - packet->right_foot.mV_heel / 1000.0f;
        right_side_.toe_fsr_.raw_reading_ = 3.4f - packet->right_foot.mV_toe / 1000.0f;
        right_side_.ps_->ankle_plantarflexion_force_N_ = packet->right_foot.mV_pull;
        right_side_.ps_->foot_imu_.quat_i_ = packet->right_foot.quatI;
        right_side_.ps_->foot_imu_.quat_j_ = packet->right_foot.quatJ;
        right_side_.ps_->foot_imu_.quat_k_ = packet->right_foot.quatK;
        right_side_.ps_->foot_imu_.quat_real_ = packet->right_foot.quatReal;
    }
}

void Exo::UsrBLEUartRxCallback(uint8_t *data, uint16_t data_size)
{
    shell_.PushPendingCommand(data, data_size);
}

void Exo::CanRxCallback(uint32_t can_id, uint8_t *data)
{
    left_side_.hip_joint_.motor_.CanRxCallBack(can_id, data);
    right_side_.hip_joint_.motor_.CanRxCallBack(can_id, data);
    left_side_.knee_joint_.motor_.CanRxCallBack(can_id, data);
    right_side_.knee_joint_.motor_.CanRxCallBack(can_id, data);
    left_side_.ankle_joint_.motor_.CanRxCallBack(can_id, data);
    right_side_.ankle_joint_.motor_.CanRxCallBack(can_id, data);
}




/* ------------------ C wrapper ------------------- */
void CallExoCanRxCallBack(Exo *ptr_exo, uint32_t can_ext_id, uint8_t *rx_data)
{
    if (ptr_exo == nullptr || rx_data == nullptr)
    {
        return;
    }
    ptr_exo->CanRxCallback(can_ext_id, rx_data);
}

void CallExoSensorUartRxCallback(Exo *ptr_exo, uint8_t *data, uint16_t data_size)
{
    if (ptr_exo == nullptr || data == nullptr)
    {
        return;
    }
    ptr_exo->SensorUartRxCallback(data, data_size);
}

void CallExoUsrBLEUartRxCallback(Exo *ptr_exo, uint8_t *data, uint16_t data_size)
{
    if (ptr_exo == nullptr || data == nullptr)
    {
        return;
    }
    ptr_exo->UsrBLEUartRxCallback(data, data_size);
}