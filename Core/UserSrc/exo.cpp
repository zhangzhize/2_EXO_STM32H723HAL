#include "exo.hpp"
#include "utils.h"
#include <string.h>
#include <algorithm>
extern "C" {
#include "arm_math.h"
}

extern uint32_t g_adc_data[3];  /**< From alt_main.cpp */


AnkleJoint::AnkleJoint(bool is_left, ExoData *pe) : pe_(pe), ps_(is_left ? &pe->left_side_ : &pe->right_side_), pj_(is_left ? &pe->left_side_.ankle_joint_ : &pe->right_side_.ankle_joint_), force_profile_generator_(), motor_(is_left ? ExoJointCanID::kLeftAnkle : ExoJointCanID::kRightAnkle), pid_(0.2f, 0.4f, 0.0f, -1.0f, 10.0f)
{
    cable_pre_tensioned_position_ = 0.0f;
    cable_tensioned_position_ = 6.0f;
    cable_released_position_ = 0.0f;
    assistance_start_phase_rad_ = 0.4 * _2PI;
    assistance_end_phase_rad_ = 0.65 * _2PI;
}

void AnkleJoint::Calibrate()
{
    if (!pj_->is_used_)
    {
        return;
    }
    /** #Todo: Implement joint calibration logic, if needed */
}

void AnkleJoint::WaitForCommunication()
{
    if (!pj_->is_used_)
    {
        return;
    }

    for (int i=0; i<3; i++)
    {
        while (motor_.feedback_flag_ == 0)
        {
            // motor_.EnableMotor();
            motor_.run_mode_ = RobstrideRunMode::kMotionMode;
            motor_.SetRunMode();
            DelayMs(100);
        }
        motor_.feedback_flag_ = 0;
    }
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
        pj_->vel_rad_s_ = motor_.speed_;
        pj_->torque_Nm_ = motor_.torque_;
    }
    else
    {
        pj_->pos_rad_ = -motor_.position_;
        pj_->vel_rad_s_ = -motor_.speed_;
        pj_->torque_Nm_ = -motor_.torque_;
    }
}

void AnkleJoint::Assist()
{
    if (!pj_->is_used_ || !ps_->is_calibration_done_)
    {
        return;
    }

    static bool assist_armed = false;
    const float phase_zero_threshold = 0.1f;
    float cable_position_ref = 0.0f;

    // float phase_rad = pj_->is_left_ ? pe_->ao_left_phase_rad_ : pe_->ao_right_phase_rad_;
    float phase_rad = pj_->is_left_ ? pe_->left_side_.percent_gait_ : pe_->right_side_.percent_gait_;
    phase_rad = phase_rad * _2PI / 100.0f;
    float phase_percent = pj_->is_left_ ? pe_->left_side_.percent_gait_ : pe_->right_side_.percent_gait_;
    uint32_t gait_event_cnt = pj_->is_left_ ? pe_->ao_left_event_cnt_ : pe_->ao_right_event_cnt_;

    if (gait_event_cnt <= 1)
    {
        assist_armed = false;
        cable_position_ref = cable_released_position_;
    }
    else
    {
        if (!assist_armed)
        {
            assist_armed = (phase_rad >= 0 ) && (phase_rad <= phase_zero_threshold);
        }
        if (assist_armed)
        {
            pe_->exo_status_ = ExoStatus::kAssisting;
            if (phase_percent >= 0.0f && phase_percent < 35.0f)
            {
                cable_position_ref = cable_pre_tensioned_position_;
            }
            else if (phase_percent > 35.0f && phase_percent < 65.0f)
            {
                cable_position_ref = cable_tensioned_position_;
            }
            else if (phase_percent >= 65.0f && phase_percent < 100.0f)
            {
                cable_position_ref = cable_released_position_;
            }
        }
        else
        {
            cable_position_ref = cable_released_position_;
        }
    }
    if (!pj_->is_left_)
    {
    }

    motor_.position_ref_ = cable_position_ref;

    motor_.torque_forward_ = 0.0f;
    motor_.speed_ref_ = 0.0f;
    motor_.motion_mode_kp_ = 15.0f;
    motor_.motion_mode_kd_ = 0.5f;
    motor_.MotionControl();
    // motor_.limit_speed_ = 6.0f;
    // motor_.PositionControlCSP();
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

void KneeJoint::WaitForCommunication()
{
    if (!pj_->is_used_)
    {
        return;
    }

    for (int i=0; i<3; i++)
    {
        while (motor_.feedback_flag_ == 0)
        {
            // motor_.EnableMotor();
            motor_.run_mode_ = RobstrideRunMode::kMotionMode;
            motor_.SetRunMode();
            DelayMs(100);
        }
        motor_.feedback_flag_ = 0;
    }
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
        pj_->vel_rad_s_ = motor_.speed_;
        pj_->torque_Nm_ = motor_.torque_;
    }
    else
    {
        pj_->pos_rad_ = -motor_.position_;
        pj_->vel_rad_s_ = -motor_.speed_;
        pj_->torque_Nm_ = -motor_.torque_;
    }
}

void KneeJoint::Assist()
{
    if (!pj_->is_used_ || !ps_->is_calibration_done_)
    {
        return;
    }

    static bool assist_armed = false;
    const float phase_zero_threshold = 0.1f;
    float force_profile = 0.0f;

    // float phase_rad = pj_->is_left_ ? pe_->ao_left_phase_rad_ : pe_->ao_right_phase_rad_;
    float phase_rad = pj_->is_left_ ? pe_->left_side_.percent_gait_ : pe_->right_side_.percent_gait_;
    phase_rad = phase_rad * _2PI / 100.0f;
    uint32_t gait_event_cnt = pj_->is_left_ ? pe_->ao_left_event_cnt_ : pe_->ao_right_event_cnt_;

    if (gait_event_cnt <= 1)
    {
        assist_armed = false;
        force_profile = 0.0f;
        /** #TODO: zero torque control (need torque sensor) */
        pe_->exo_status_ = ExoStatus::kReady;
    }
    else
    {
        if (!assist_armed)
        {
            assist_armed = (phase_rad >= 0 ) && (phase_rad <= phase_zero_threshold);
        }
        if (assist_armed)
        {
            force_profile = force_profile_generator_.GetForceProfile(phase_rad, pj_->pos_rad_, pj_->vel_rad_s_);
            pe_->exo_status_ = ExoStatus::kAssisting;
        }
        else
        {
            force_profile = 0.0f;
        }
    }

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
    disturbance_observer_.UpdateObserver(pj_->pos_rad_, pj_->torque_Nm_ / M0_);

    float hat_disturbance = - M0_ * disturbance_observer_.hat_x2_;
    float dot_tilde_q = dot_q_ref_ - pj_->vel_rad_s_;
    float tilde_q = q_ref_ - pj_->pos_rad_;

    motor_.torque_forward_ = M0_ * ddot_q_ref_ + M0_ / Md_ * (Bd_ * dot_tilde_q + Kd_ * tilde_q - hat_disturbance) + hat_disturbance;
    motor_.position_ref_ = 0;
    motor_.speed_ref_ = 0;
    motor_.motion_mode_kp_ = 0;
    motor_.motion_mode_kd_ = 0;
    motor_.MotionControl();
}



Side::Side(bool is_left, ExoData *pe) : pe_(pe), ps_(is_left ? &pe_->left_side_ : &pe_->right_side_), heel_fsr_(is_left ? kFsrIdLeftHeel : kFsrIdRightHeel), toe_fsr_(is_left ? kFsrIdLeftToe : kFsrIdRightToe), knee_joint_(is_left, pe), ankle_joint_(is_left, pe)
{
}

void Side::Calibrate()
{
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

    knee_joint_.Read();
    ankle_joint_.Read();
    heel_fsr_.Read();
    toe_fsr_.Read();
}

void Side::Assist()
{
    if (!ps_->is_used_)
    {
        return;
    }

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
        pe_->exo_status_ = ExoStatus::kCalibration;
        ps_->do_calibration_toe_fsr_ = toe_fsr_.Calibrate(ps_->do_calibration_toe_fsr_);
    }
    else if (ps_->do_calibration_refinement_toe_fsr_)
    {
        pe_->exo_status_ = ExoStatus::kCalibration;
        ps_->do_calibration_refinement_toe_fsr_ = toe_fsr_.RefineCalibration(ps_->do_calibration_refinement_toe_fsr_);
    }

    if (ps_->do_calibration_heel_fsr_)
    {
        pe_->exo_status_ = ExoStatus::kCalibration;
        ps_->do_calibration_heel_fsr_ = heel_fsr_.Calibrate(ps_->do_calibration_heel_fsr_);
    }
    else if (ps_->do_calibration_refinement_heel_fsr_)
    {
        pe_->exo_status_ = ExoStatus::kCalibration;
        ps_->do_calibration_refinement_heel_fsr_ = heel_fsr_.RefineCalibration(ps_->do_calibration_refinement_heel_fsr_);
    }
}

void Side::EstimateGait()
{
    if (!ps_->is_used_)
    {
        return;
    }

    ps_->heel_contact_state_ = heel_fsr_.GetGroundContact();
    ps_->toe_contact_state_ = toe_fsr_.GetGroundContact();

    /**< Now use heel fsr only */
    // ps_->ground_strike_ = (!ps_->prev_heel_contact_state_ && !ps_->prev_toe_contact_state_) && ((ps_->heel_contact_state_ && !ps_->prev_heel_contact_state_) || (ps_->toe_contact_state_ && !ps_->prev_toe_contact_state_));
    ps_->ground_strike_ = (!ps_->prev_heel_contact_state_) && (ps_->heel_contact_state_ && !ps_->prev_heel_contact_state_);
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
    knee_joint_.motor_.StopMotor(0);
    ankle_joint_.motor_.StopMotor(0);
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

    else if ((step_time <= (ps_->expected_duration_window_upper_coeff_ * *max_val)) & (step_time >= (ps_->expected_duration_window_lower_coeff_ * *min_val)))
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
    else if ((stance_time <= (ps_->expected_duration_window_upper_coeff_ * *max_val)) & (stance_time >= (ps_->expected_duration_window_lower_coeff_ * *min_val)))
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
    else if ((swing_time <= (ps_->expected_duration_window_upper_coeff_ * *max_val)) & (swing_time >= (ps_->expected_duration_window_lower_coeff_ * *min_val)))
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

Exo::Exo(ExoData *pe) : pe_(pe), status_led_(), adaptive_oscilator_(), left_side_(true, pe), right_side_(false, pe)
{
}

void Exo::Initialize()
{
    /** 等待上电 */
    uint8_t exo_status_idx = 0;
    while (pe_->battery_voltage_ < 19.0f)
    {
        ReadBatVol();
        DelayMs(100);

        pe_->exo_status_ = ExoStatus::kErrorBatteryLowVoltage;
        exo_status_idx = static_cast<uint8_t>(pe_->exo_status_);
        status_led_.UpdateColor(exo_status_idx > 8 ? 8 : exo_status_idx);
    }
    /** 进入标定阶段 */
    pe_->exo_status_ = ExoStatus::kCalibration;
    exo_status_idx = static_cast<uint8_t>(pe_->exo_status_);
    status_led_.UpdateColor(exo_status_idx > 8 ? 8 : exo_status_idx);

    /** 调试: 设置施密特触发器阈值, 用于判断足跟着地事件. */
    left_side_.heel_fsr_.SetContactThresholds(0.15f, 0.25f);
    left_side_.toe_fsr_.SetContactThresholds(0.15f, 0.25f);
    right_side_.heel_fsr_.SetContactThresholds(0.15f, 0.25f);
    right_side_.toe_fsr_.SetContactThresholds(0.15f, 0.25f);
    // left_side_.heel_fsr_.calibration_refinement_max_ = 2.45f;
    // left_side_.heel_fsr_.calibration_refinement_min_ = 0.15f;
    // right_side_.heel_fsr_.calibration_refinement_max_ = 2.45f;
    // right_side_.heel_fsr_.calibration_refinement_min_ = 0.15f;
    pe_->left_side_.do_calibration_heel_fsr_ = true;
    pe_->left_side_.do_calibration_toe_fsr_ = false;
    pe_->left_side_.do_calibration_refinement_heel_fsr_ = true;
    pe_->left_side_.do_calibration_refinement_toe_fsr_ = false;
    pe_->right_side_.do_calibration_heel_fsr_ = true;
    pe_->right_side_.do_calibration_toe_fsr_ = false;
    pe_->right_side_.do_calibration_refinement_heel_fsr_ = true;
    pe_->right_side_.do_calibration_refinement_toe_fsr_ = false;

    /** 调试: 选择助力的关节 */
    pe_->left_side_.knee_joint_.is_used_ = false;
    pe_->right_side_.knee_joint_.is_used_ = false;
    pe_->left_side_.ankle_joint_.is_used_ = false;
    pe_->right_side_.ankle_joint_.is_used_ = false;

    /** 调试: 调助力大小 */
    pe_->user_weight_kg_ = 30.0f;
    right_side_.knee_joint_.force_profile_generator_.stiffness_ = 0.5f;

    left_side_.ankle_joint_.cable_released_position_ = 0.0f;
    left_side_.ankle_joint_.cable_pre_tensioned_position_ = 0.5f;
    left_side_.ankle_joint_.cable_tensioned_position_ = 1.5f;
    left_side_.ankle_joint_.assistance_start_phase_rad_ = 0.4f * _2PI;
    left_side_.ankle_joint_.assistance_end_phase_rad_ = 0.65f * _2PI;

    /** 跟电机建立通信 */
    left_side_.knee_joint_.WaitForCommunication();
    left_side_.ankle_joint_.WaitForCommunication();
    right_side_.knee_joint_.WaitForCommunication();
    right_side_.ankle_joint_.WaitForCommunication();
}

void Exo::Run()
{
    Calibrate();
    Read();
    Estimate();
    Assist();

    /** #TODO: check status */
    if (pe_->battery_voltage_ < 19.0f)
    {
        pe_->exo_status_ = ExoStatus::kErrorBatteryLowVoltage;
    }

    if (left_side_.knee_joint_.motor_.error_code_ != 0 || left_side_.knee_joint_.motor_.fault_code_ != 0 || right_side_.knee_joint_.motor_.error_code_ != 0 || right_side_.knee_joint_.motor_.fault_code_ != 0)
    {
        pe_->exo_status_ = ExoStatus::kErrorRobstride;
    }

    uint8_t exo_status_idx = static_cast<uint8_t>(pe_->exo_status_);
    status_led_.UpdateColor(exo_status_idx > 8 ? 8 : exo_status_idx);

    // if (exo_status_idx >= 8)
    // {
    //     Shutdown();
    //     while (1)
    //     {
    //         DelayMs(1000);
    //     }
    // }
}

void Exo::Calibrate()
{
    left_side_.Calibrate();
    right_side_.Calibrate();
    if (pe_->left_side_.is_calibration_done_ && pe_->right_side_.is_calibration_done_)
    {
        pe_->exo_status_ = ExoStatus::kReady;
    }
}

void Exo::Read()
{
    ReadBatVol();
    left_side_.Read();
    right_side_.Read();
}

void Exo::Estimate()
{
    /** #TODO 估计在执行的动作: 平地走, 上下坡, 上下楼梯, 坐立转换 */


    /**< 平地走：估计步态相位 */
    left_side_.EstimateGait();
    right_side_.EstimateGait();

    /**< 双足膝关节角度差 */
    adaptive_oscilator_.UpdateGaitPhase(pe_->left_side_.knee_joint_.pos_rad_, pe_->right_side_.knee_joint_.pos_rad_, pe_->left_side_.knee_joint_.vel_rad_s_, pe_->right_side_.knee_joint_.vel_rad_s_, pe_->left_side_.heel_contact_state_, pe_->right_side_.heel_contact_state_);
    pe_->ao_left_phase_rad_ = adaptive_oscilator_.left_phi_comp_rad_;
    pe_->ao_right_phase_rad_ = adaptive_oscilator_.right_phi_comp_rad_;
    pe_->ao_left_event_cnt_ = adaptive_oscilator_.left_event_cnt_;
    pe_->ao_right_event_cnt_ = adaptive_oscilator_.right_event_cnt_;
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

void Exo::ReadBatVol()
{
    pe_->battery_voltage_ = (g_adc_data[0] * 3.3f / 65535) * 11;
}

void Exo::UartRxCallback(uint8_t *data, uint16_t data_size)
{
    if (data[0] == 0x01 && data_size >= 17)
    {
        int32_t leftfoot_mv_ain0;
        int32_t leftfoot_mv_ain1;
        int32_t rightfoot_mv_ain0;
        int32_t rightfoot_mv_ain1;
        memcpy(&leftfoot_mv_ain0, data + 1, sizeof(int32_t));
        memcpy(&leftfoot_mv_ain1, data + 1 + sizeof(int32_t), sizeof(int32_t));
        memcpy(&rightfoot_mv_ain0, data + 1 + 2 * sizeof(int32_t), sizeof(int32_t));
        memcpy(&rightfoot_mv_ain1, data + 1 + 3 * sizeof(int32_t), sizeof(int32_t));

        fsr_voltages[0] = 3.4f - leftfoot_mv_ain0 / 1000.0f;
        fsr_voltages[1] = 3.4f - leftfoot_mv_ain1 / 1000.0f;
        fsr_voltages[2] = 3.4f - rightfoot_mv_ain0 / 1000.0f;
        fsr_voltages[3] = 3.4f - rightfoot_mv_ain1 / 1000.0f;
        if (fsr_voltages[0] < 0.0f) fsr_voltages[0] = 0.0f;
        if (fsr_voltages[1] < 0.0f) fsr_voltages[1] = 0.0f;
        if (fsr_voltages[2] < 0.0f) fsr_voltages[2] = 0.0f;
        if (fsr_voltages[3] < 0.0f) fsr_voltages[3] = 0.0f;
    }
    else if (data[0] == 0x02 && data_size >= 5)
    {
        float user_weight_kg;
        memcpy(&user_weight_kg, data + 1, sizeof(float));
        if (user_weight_kg >= 20.0f && user_weight_kg <= 120.0f)
        {
            pe_->user_weight_kg_ = user_weight_kg;
        }
    }
}

void Exo::CanRxCallback(uint32_t can_id, uint8_t *data)
{
    left_side_.knee_joint_.motor_.CanRxCallBack(can_id, data);
    left_side_.ankle_joint_.motor_.CanRxCallBack(can_id, data);
    right_side_.knee_joint_.motor_.CanRxCallBack(can_id, data);
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

void CallExoUartRxCallBack(Exo *ptr_exo, uint8_t *data, uint16_t data_size)
{
    if (ptr_exo == nullptr || data == nullptr)
    {
        return;
    }
    ptr_exo->UartRxCallback(data, data_size);
}
