#include "exo.hpp"
#include "utils.h"
#include <algorithm>
extern "C" {
#include "arm_math.h"
}
#include "dwt.h"

extern uint32_t g_adc_data[3];  /**< definition in alt_main.cpp */

volatile float force_ref_temp = 0.0f;

void AnkleJoint::Calibrate()
{
    if (!pj_.is_used_ || pj_.is_calibrated_) return;
}

bool AnkleJoint::IsMotorConnect()
{
    if (!pj_.is_used_) return true;

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
    if (!pj_.is_used_) return;

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
  if (!pj_.is_used_) return;

  if (pj_.is_left_) {
    pj_.pos_rad_ = motor_.position_;
    pj_.vel_radps_ = motor_.speed_;
    pj_.tor_Nm_ = motor_.torque_;
  } else {
    pj_.pos_rad_ = -motor_.position_;
    pj_.vel_radps_ = -motor_.speed_;
    pj_.tor_Nm_ = -motor_.torque_;
  }
}

void AnkleJoint::Assist()
{
    if (!pj_.is_used_) return;

    /** 1. 获取 FSR 算出的相位百分比 (0.0f~100.0f) */
    float phase_percent = pj_.is_left_ ? pe_.left_side_.fsr_gait_data_.percent_gait_ : pe_.right_side_.fsr_gait_data_.percent_gait_;

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

    if (!pj_.is_left_)
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

void KneeJoint::Calibrate()
{
    if (!pj_.is_used_ || pj_.is_calibrated_) return;
}

bool KneeJoint::IsMotorConnect()
{
    if (!pj_.is_used_) return true;

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
    if (!pj_.is_used_) return;

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
    if (!pj_.is_used_) return;

    if (pj_.is_left_)
    {
        pj_.pos_rad_ = motor_.position_;
        pj_.vel_radps_ = motor_.speed_;
        pj_.tor_Nm_ = motor_.torque_;
    }
    else
    {
        pj_.pos_rad_ = -motor_.position_;
        pj_.vel_radps_ = -motor_.speed_;
        pj_.tor_Nm_ = -motor_.torque_;
    }
}

void KneeJoint::Assist()
{
    if (!pj_.is_used_) return;

    float force_profile = 0.0f;

    // float phase_rad = pj_.is_left_ ? pe_.ao_left_phase_rad_ : pe_.ao_right_phase_rad_;
    float phase_rad = pj_.is_left_ ? pe_.left_side_.fsr_gait_data_.percent_gait_ : pe_.right_side_.fsr_gait_data_.percent_gait_;
    phase_rad = phase_rad * _2PI / 100.0f;
    // uint32_t gait_event_cnt = pj_.is_left_ ? pe_.left_event_cnt_ : pe_.right_event_cnt_;

    // float phase_rad = pj_.is_left_ ? pe_.ao_left_phase_rad_ : pe_.ao_right_phase_rad_;
    // float phase_percent = phase_rad * _2PI / 100.0f;

    // float phase_percent = pj_.is_left_ ? pe_.left_side_.percent_gait_ : pe_.right_side_.percent_gait_;

    // uint32_t gait_event_cnt = pj_.is_left_ ? pe_.left_event_cnt_ : pe_.right_event_cnt_;

    force_profile = force_profile_generator_.GetForceProfile(phase_rad, pj_.pos_rad_, pj_.vel_radps_);
    if (!pj_.is_left_)
    {
        force_profile = -force_profile;
    }
    motor_.torque_forward_ = force_profile * pe_.user_weight_kg_;
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
    static float Md_ = 1.0f;
    static float Bd_ = 1.0f;
    static float Kd_ = 1.0f;

    if (!pj_.is_used_)
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
    disturbance_observer_.UpdateObserver(pj_.pos_rad_, pj_.tor_Nm_ / M0_);

    float hat_disturbance = - M0_ * disturbance_observer_.hat_x2_;
    float dot_tilde_q = dot_q_ref_ - pj_.vel_radps_;
    float tilde_q = q_ref_ - pj_.pos_rad_;

    motor_.torque_forward_ = M0_ * ddot_q_ref_ + M0_ / Md_ * (Bd_ * dot_tilde_q + Kd_ * tilde_q - hat_disturbance) + hat_disturbance;
    motor_.position_ref_ = 0;
    motor_.speed_ref_ = 0;
    motor_.motion_mode_kp_ = 0;
    motor_.motion_mode_kd_ = 0;
    motor_.MotionControl();
}

void KneeSeaJoint::Calibrate()
{
    if (!pj_.is_used_ || pj_.is_calibrated_) return;

    static uint8_t near_zero_cnt = 0;
    pj_.pos_ref_rad_ = 0;
    joint_pos_pid_.output_limit_ = motor_.max_iqref_amp_ / 5.0f;
    JointPosControl();

    if (pj_.pos_rad_ > -0.01f && pj_.pos_rad_ < 0.01f)
    {
        if (near_zero_cnt++ > 100)
        {
            near_zero_cnt = 0;
            joint_pos_pid_.output_limit_ = motor_.max_iqref_amp_;
            pj_.pos_slider_offset_mm_ = pj_.pos_slider_mm_;
            pj_.is_calibrated_ = true;
        }
    }
    else
    {
        near_zero_cnt = 0;
    }
}

void KneeSeaJoint::Read()
{
    if (!pj_.is_used_) return;

    /** #TODO 将电机的数据和磁栅尺的数据进行处理 */

    /** 滑块位移 = 电机输出转动角度 * 螺母导程 - 偏置 */
    pj_.pos_slider_mm_ = pj_.screw_lead_rad2mm_ * motor_.shaft_pos_feedback_rad_ - pj_.pos_slider_offset_mm_;
    pj_.vel_slider_mmps_ = pj_.screw_lead_rad2mm_ * motor_.shaft_speed_feedback_radps_;

    /** sea输出位移 = 磁栅尺反馈位置 - 偏置 */
    pj_.pos_linear_encoder_mm_ = mag_encoder_.absolute_position_mm_ - pj_.pos_linear_encoder_offset_mm_;
    pj_.vel_linear_encoder_mmps_ = 0.0f;  /** #HACK 暂时不需要  */

    /** 弹簧力 = 2 * 刚度 * 压缩长度 */
    pj_.pos_bias_mm_ = pj_.pos_slider_mm_ - pj_.pos_linear_encoder_mm_;
    pj_.force_spring_N_ = pj_.spring_stiffness_Npmm_ * pj_.pos_bias_mm_;

    /** 关节的pos_rad_, vel_radps, tor_Nm_ 需要进一步计算/测量(因为连杆和腿部对不准) */
    pj_.pos_rad_ = (pj_.pos_linear_encoder_mm_) / (pj_.max_pos_linear_encoder_mm_ - pj_.pos_linear_encoder_offset_mm_) * _PI_2;

    mag_encoder_.SendRequest();
}

bool KneeSeaJoint::IsMotorConnect()
{
    if (!pj_.is_used_) return true;

    return true; /** 无需主动连接通信 */
}

void KneeSeaJoint::Shutdown()
{
    motor_.DisableMotor();
}

void KneeSeaJoint::Standby()
{
    if (!pj_.is_used_ || !pj_.is_calibrated_) return;

    float force_profile = 0.0f;
    /** SEA驱动左右同向 */
    // force_profile = pj_.is_left_ ? force_profile : -force_profile;

    /** 弹簧零力控制 */
    pj_.force_spring_ref_N_ = force_profile * pe_.user_weight_kg_;
    SpringForceControl(); 
}

void KneeSeaJoint::Assist()
{
    if (!pj_.is_used_) return;

    float force_profile = 0.0f;

    float phase_percent = pj_.is_left_ ? pe_.left_side_.fsr_gait_data_.percent_gait_ : pe_.right_side_.fsr_gait_data_.percent_gait_;
    float phase_rad = phase_percent * _2PI / 100.0f;

    /** #HACK 先走几步再助力? */
    force_profile = force_profile_generator_.GetForceProfile(phase_rad, pj_.pos_rad_, pj_.vel_radps_);

    pj_.force_spring_ref_N_ = force_profile * pe_.user_weight_kg_;

    force_ref_temp = pj_.force_spring_ref_N_;
    SpringForceControl();
    // Standby();
}

void KneeSeaJoint::JointPosControl()
{
    if (ctrl_mode_ != CtrlMode::kJointPos)
    {
        joint_pos_pid_.ResetError();
        spring_force_pid_.ResetError();
        ctrl_mode_ = CtrlMode::kJointPos;
    }
    
    float pos_err_rad = pj_.pos_ref_rad_ - pj_.pos_rad_;
    motor_.rotor_iq_reference_amp_ = joint_pos_pid_(pos_err_rad);

    motor_.CurrentControl();
}

void KneeSeaJoint::SpringForceControl()
{
    if (ctrl_mode_ != CtrlMode::kSpringForce)
    {
        joint_pos_pid_.ResetError();
        spring_force_pid_.ResetError();
        ctrl_mode_ = CtrlMode::kSpringForce;
    }

    float force_err_N = pj_.force_spring_ref_N_ - pj_.force_spring_N_;
    motor_.rotor_iq_reference_amp_ = spring_force_pid_(force_err_N);

    motor_.CurrentControl();
}


void HipJoint::Calibrate()
{
    if (!pj_.is_used_ || pj_.is_calibrated_) return;
}

void HipJoint::Read()
{
    if (!pj_.is_used_) return;

    if (pj_.is_left_)
    {
        pj_.pos_rad_ = motor_.feedback_.pos_rad_;
        pj_.vel_radps_ = motor_.feedback_.vel_radps_;
        pj_.tor_Nm_ = motor_.feedback_.tor_Nm_;
    }
    else
    {
        pj_.pos_rad_ = -motor_.feedback_.pos_rad_;
        pj_.vel_radps_ = -motor_.feedback_.vel_radps_;
        pj_.tor_Nm_ = -motor_.feedback_.tor_Nm_;
    }
}

bool HipJoint::IsMotorConnect()
{
    if (!pj_.is_used_) return true;

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
    if (!pj_.is_used_) return;

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
    if (!pj_.is_used_) return;

    const float posL_now = pe_.left_side_.hip_joint_.pos_rad_;
    const float posR_now = pe_.right_side_.hip_joint_.pos_rad_;
    const uint32_t now_ms = GetSysTimeMs();

    if (pj_.is_left_)
    {
        g_hip_ff.UpdateAndCompute(now_ms, posL_now, posR_now);
    }
    const float tau_me = pj_.is_left_ ? g_hip_ff.tau_left : g_hip_ff.tau_right;

    motor_.ctrl_param_.mode_id_ = DMMotorModeID::kMIT;
    motor_.ctrl_param_.kp_set_ = 0.0f;
    motor_.ctrl_param_.kd_set_ = 0.0f;
    motor_.ctrl_param_.pos_set_rad_ = 0.0f;
    motor_.ctrl_param_.vel_set_radps_ = 0.0f;
    motor_.ctrl_param_.tor_set_Nm_ = tau_me;
    motor_.MitControl();
    // motor_.EnableMotor();
}

void FsrGaitEstimator::Calibrate()
{
    if (!gait_data_.is_used_ || gait_data_.is_calibrated_) return;

    ProcessCalibration(gait_data_.heel_, gait_data_.do_calibration_heel_fsr_, gait_data_.do_calibration_refinement_heel_fsr_);
    ProcessCalibration(gait_data_.toe_, gait_data_.do_calibration_toe_fsr_, gait_data_.do_calibration_refinement_toe_fsr_);

    gait_data_.is_calibrated_ = !(gait_data_.do_calibration_heel_fsr_ || gait_data_.do_calibration_refinement_heel_fsr_ || gait_data_.do_calibration_toe_fsr_ || gait_data_.do_calibration_refinement_toe_fsr_);
}

void FsrGaitEstimator::Update() {
    if (!gait_data_.is_used_) return;

    // 1. 先由引擎处理硬件原始数据，并更新校准标志位
    ProcessSensorUpdate(gait_data_.heel_);
    ProcessSensorUpdate(gait_data_.toe_);

    // 2. 获取处理后的状态，进入原有的步态解算逻辑
    gait_data_.heel_contact_state_ = gait_data_.heel_.ground_contact;
    gait_data_.toe_contact_state_ = gait_data_.toe_.ground_contact;
    
    /** #HACK 暂时只用足跟fsr检测strike */
    // gait_data_.ground_strike_ = (!gait_data_.prev_heel_contact_state_ && !gait_data_.prev_toe_contact_state_) && ((gait_data_.heel_contact_state_ && !gait_data_.prev_heel_contact_state_) || (gait_data_.toe_contact_state_ && !gait_data_.prev_toe_contact_state_));
    gait_data_.ground_strike_ = (gait_data_.heel_contact_state_ && !gait_data_.prev_heel_contact_state_);
    gait_data_.toe_off_       = (!gait_data_.toe_contact_state_ && gait_data_.prev_toe_contact_state_);
    gait_data_.toe_strike_    = (gait_data_.toe_contact_state_ && !gait_data_.prev_toe_contact_state_);

    gait_data_.prev_heel_contact_state_ = gait_data_.heel_contact_state_;
    gait_data_.prev_toe_contact_state_  = gait_data_.toe_contact_state_;

    /** 3. 步态事件时间戳与历史窗口更新 */
    uint32_t now_ms = GetSysTimeMs();
    if (gait_data_.ground_strike_)
    {
        gait_data_.prev_ground_strike_timestamp_ = gait_data_.ground_strike_timestamp_;
        gait_data_.ground_strike_timestamp_ = now_ms;
        gait_data_.expected_step_duration_ = UpdateExpectedDuration();
    }
    if (gait_data_.toe_strike_)
    {
        gait_data_.prev_toe_strike_timestamp_ = gait_data_.toe_strike_timestamp_;
        gait_data_.toe_strike_timestamp_ = now_ms;
        gait_data_.expected_swing_duration_ = UpdateExpectedSwingDuration();
    }
    if (gait_data_.toe_off_)
    {
        gait_data_.prev_toe_off_timestamp_ = gait_data_.toe_off_timestamp_;
        gait_data_.toe_off_timestamp_ = now_ms;
        gait_data_.expected_stance_duration_ = UpdateExpectedStanceDuration();
    }

    // 4. 百分比解算 (Phase Calculation)
    if (gait_data_.expected_step_duration_ > 0.0f)
    {
        gait_data_.percent_gait_ = 100.0f * ((static_cast<float>(now_ms) - static_cast<float>(gait_data_.ground_strike_timestamp_)) / gait_data_.expected_step_duration_);
        if (gait_data_.percent_gait_ > 100.0f)
            gait_data_.percent_gait_ = 100.0f;
    }

    if (gait_data_.expected_stance_duration_ > 0.0f)
    {
        gait_data_.percent_stance_ = 100.0f * ((static_cast<float>(now_ms) - static_cast<float>(gait_data_.toe_strike_timestamp_)) / gait_data_.expected_stance_duration_);
        if (gait_data_.percent_stance_ > 100.0f)
            gait_data_.percent_stance_ = 100.0f;
    }
    if (!gait_data_.heel_contact_state_ && !gait_data_.toe_contact_state_)
    {
        gait_data_.percent_stance_ = 0.0f;
    }

    if (gait_data_.expected_swing_duration_ > 0.0f)
    {
        gait_data_.percent_swing_ = 100.0f * ((static_cast<float>(now_ms) - static_cast<float>(gait_data_.toe_off_timestamp_)) / gait_data_.expected_swing_duration_);
        if (gait_data_.percent_swing_ > 100.0f)
            gait_data_.percent_swing_ = 100.0f;
    }
    if (gait_data_.heel_contact_state_ || gait_data_.toe_contact_state_)
    {
        gait_data_.percent_swing_ = 0.0f;
    }
}


void FsrGaitEstimator::Reset()
{
    for (int i = 0; i < FsrGaitData::kNumStepsAvg; i++)
    {
        gait_data_.step_times_[i] = 0;
        gait_data_.stance_times_[i] = 0;
        gait_data_.swing_times_[i] = 0;
    }
}

void FsrGaitEstimator::ProcessCalibration(FsrSensorData& sensor, bool& do_calibrate, bool& do_refinement)
{
    uint32_t now_ms = GetSysTimeMs();

    /** 1. 基础校准阶段 */
    if (do_calibrate && !sensor.last_do_calibrate)
    {
        sensor.calibration_start_sys_ms = now_ms;
        sensor.calibration_max = sensor.raw_reading;
        sensor.calibration_min = sensor.calibration_max;
    }

    if (do_calibrate && (now_ms - sensor.calibration_start_sys_ms <= FsrSensorData::kCalibrationDurationMs))
    {
        sensor.calibration_max = _max(sensor.calibration_max, sensor.raw_reading);
        sensor.calibration_min = _min(sensor.calibration_min, sensor.raw_reading);
    }
    else if (do_calibrate)
    {
        do_calibrate = false;
    }
    sensor.last_do_calibrate = do_calibrate;

     /** 2. 精细校准阶段 (Refinement) */
    if (do_refinement && !sensor.last_do_refinement)
    {
        sensor.refinement_step_count= 0;
        sensor.step_max = (sensor.calibration_max + sensor.calibration_min) / 2.0f;
        sensor.step_min = sensor.step_max;
        sensor.step_max_sum = 0.0f;
        sensor.step_min_sum = 0.0f;
    }

    if (do_refinement)
    {
        if (sensor.refinement_step_count < FsrSensorData::kNumRefinementSteps)
        {
            sensor.step_max = _max(sensor.step_max, sensor.raw_reading);
            sensor.step_min = _min(sensor.step_min, sensor.raw_reading);

            bool last_state = sensor.ground_contact_during_refinement;
            float lower = FsrSensorData::kSchmittLowerThresholdRefinement * (sensor.calibration_max - sensor.calibration_min) + sensor.calibration_min;
            float upper = FsrSensorData::kSchmittUpperThresholdRefinement * (sensor.calibration_max - sensor.calibration_min) + sensor.calibration_min;

            sensor.ground_contact_during_refinement = SchmittTrigger(sensor.raw_reading, last_state, lower, upper);

            if (sensor.ground_contact_during_refinement && !last_state)
            {
                sensor.step_max_sum += sensor.step_max;
                sensor.step_min_sum += sensor.step_min;
                float mid = (sensor.calibration_max + sensor.calibration_min) / 2.0f;
                sensor.step_max = mid;
                sensor.step_min = mid;
                sensor.refinement_step_count++;
            }
        }
        else
        {
            if (sensor.refinement_step_count > 0)
            {
                sensor.calibration_refinement_max = sensor.step_max_sum / FsrSensorData::kNumRefinementSteps;
                sensor.calibration_refinement_min = sensor.step_min_sum / FsrSensorData::kNumRefinementSteps;
            }
            else
            {
                sensor.calibration_refinement_max = sensor.calibration_max;
                sensor.calibration_refinement_min = sensor.calibration_min;
            }
            do_refinement = false;
        }
    }
    sensor.last_do_refinement = do_refinement;
}

void FsrGaitEstimator::ProcessSensorUpdate(FsrSensorData& sensor)
{
    /** 计算归一化读取值 (Read) */
    if (sensor.calibration_refinement_max > sensor.calibration_refinement_min + 1e-6f)
    {
        sensor.calibrated_reading = (sensor.raw_reading - sensor.calibration_refinement_min) / (sensor.calibration_refinement_max - sensor.calibration_refinement_min);
    }
    else if (sensor.calibration_max > sensor.calibration_min + 1e-6f)
    {
        sensor.calibrated_reading = (sensor.raw_reading - sensor.calibration_min) / (sensor.calibration_max - sensor.calibration_min);
    }
    else
    {
        sensor.calibrated_reading = sensor.raw_reading;
    }

    if (sensor.calibration_refinement_max > sensor.calibration_refinement_min + 1e-6f || sensor.calibration_max > sensor.calibration_min + 1e-6f)
    {
        if (sensor.calibrated_reading < 0.0f) sensor.calibrated_reading = 0.0f;
        if (sensor.calibrated_reading > 1.0f) sensor.calibrated_reading = 1.0f;
    }

    /** 更新接触状态 (Ground Contact) */
    if (sensor.calibration_refinement_max > sensor.calibration_refinement_min + 1e-6f)
    {
        sensor.ground_contact = SchmittTrigger(sensor.calibrated_reading, sensor.ground_contact, sensor.schmitt_lower_threshold_calc_contact, sensor.schmitt_upper_threshold_calc_contact);
    }
}


float FsrGaitEstimator::UpdateExpectedDuration()
{
    uint32_t step_time = gait_data_.ground_strike_timestamp_ - gait_data_.prev_ground_strike_timestamp_;
    float expected_step_duration = gait_data_.expected_step_duration_;

    if (0 == gait_data_.prev_ground_strike_timestamp_) return expected_step_duration;

    uint8_t num_uninitialized = 0;
    for (int i = 0; i < FsrGaitData::kNumStepsAvg; i++)
    {
        num_uninitialized += (gait_data_.step_times_[i] == 0);
    }

    uint32_t *max_val = std::max_element(gait_data_.step_times_, gait_data_.step_times_ + FsrGaitData::kNumStepsAvg);
    uint32_t *min_val = std::min_element(gait_data_.step_times_, gait_data_.step_times_ + FsrGaitData::kNumStepsAvg);

    if  (num_uninitialized > 0)
    {
        for (int i = (FsrGaitData::kNumStepsAvg - 1); i>0; i--)
        {
            gait_data_.step_times_[i] = gait_data_.step_times_[i-1];
        }
        gait_data_.step_times_[0] = step_time;
    }

    else if ((step_time <= (gait_data_.expected_duration_window_upper_coeff_ * *max_val)) && (step_time >= (gait_data_.expected_duration_window_lower_coeff_ * *min_val)))
    {
        int sum_step_times = step_time;
        for (int i = (FsrGaitData::kNumStepsAvg - 1); i>0; i--)
        {
            sum_step_times += gait_data_.step_times_[i-1];
            gait_data_.step_times_[i] = gait_data_.step_times_[i-1];
        }
        gait_data_.step_times_[0] = step_time;

        expected_step_duration = sum_step_times / FsrGaitData::kNumStepsAvg;
    }
    return expected_step_duration;
}

float FsrGaitEstimator::UpdateExpectedStanceDuration()
{
    uint32_t stance_time = gait_data_.toe_off_timestamp_ - gait_data_.toe_strike_timestamp_;
    float expected_stance_duration = gait_data_.expected_stance_duration_;

    if (0 == gait_data_.prev_toe_strike_timestamp_) return expected_stance_duration;
    
    uint8_t num_uninitialized = 0;
    for (int i = 0; i < FsrGaitData::kNumStepsAvg; i++)
    {
        num_uninitialized += (gait_data_.stance_times_[i] == 0);
    }

    uint32_t *max_val = std::max_element(gait_data_.stance_times_, gait_data_.stance_times_ + FsrGaitData::kNumStepsAvg);
    uint32_t *min_val = std::min_element(gait_data_.stance_times_, gait_data_.stance_times_ + FsrGaitData::kNumStepsAvg);

    if (num_uninitialized > 0)
    {
        for (int i = (FsrGaitData::kNumStepsAvg - 1); i>0; i--)
        {
            gait_data_.stance_times_[i] = gait_data_.stance_times_[i - 1];
        }
        gait_data_.stance_times_[0] = stance_time;
    }
    else if ((stance_time <= (gait_data_.expected_duration_window_upper_coeff_ * *max_val)) && (stance_time >= (gait_data_.expected_duration_window_lower_coeff_ * *min_val)))
    {
        int sum_stance_times = stance_time;
        for (int i = (FsrGaitData::kNumStepsAvg - 1); i>0; i--)
        {
            sum_stance_times += gait_data_.stance_times_[i - 1];
            gait_data_.stance_times_[i] = gait_data_.stance_times_[i - 1];
        }
        gait_data_.stance_times_[0] = stance_time;

        expected_stance_duration = sum_stance_times / FsrGaitData::kNumStepsAvg;
    }
    return expected_stance_duration;
}

float FsrGaitEstimator::UpdateExpectedSwingDuration()
{
    uint32_t swing_time = gait_data_.toe_strike_timestamp_ - gait_data_.toe_off_timestamp_;
    float expected_swing_duration = gait_data_.expected_swing_duration_;

    if (0 == gait_data_.prev_toe_off_timestamp_)
    {
        return expected_swing_duration;
    }

    uint8_t num_uninitialized = 0;

    for (int i = 0; i < FsrGaitData::kNumStepsAvg; i++)
    {
        num_uninitialized += (gait_data_.swing_times_[i] == 0);
    }

    uint32_t *max_val = std::max_element(gait_data_.swing_times_, gait_data_.swing_times_ + FsrGaitData::kNumStepsAvg);
    uint32_t *min_val = std::min_element(gait_data_.swing_times_, gait_data_.swing_times_ + FsrGaitData::kNumStepsAvg);

    if (num_uninitialized > 0)
    {
        for (int i = (FsrGaitData::kNumStepsAvg - 1); i>0; i--)
        {
            gait_data_.swing_times_[i] = gait_data_.swing_times_[i - 1];
        }
        gait_data_.swing_times_[0] = swing_time;
    }
    else if ((swing_time <= (gait_data_.expected_duration_window_upper_coeff_ * *max_val)) && (swing_time >= (gait_data_.expected_duration_window_lower_coeff_ * *min_val)))
    {
        int sum_swing_times = swing_time;
        for (int i = (FsrGaitData::kNumStepsAvg - 1); i>0; i--)
        {
            sum_swing_times += gait_data_.swing_times_[i - 1];
            gait_data_.swing_times_[i] = gait_data_.swing_times_[i - 1];
        }
        gait_data_.swing_times_[0] = swing_time;

        expected_swing_duration = sum_swing_times / FsrGaitData::kNumStepsAvg;
    }
    return expected_swing_duration;
}


void Side::Calibrate()
{
    /** #TODO 实现关节的标定 */
    hip_joint_.Calibrate();
    knee_joint_.Calibrate();
    ankle_joint_.Calibrate();
    knee_sea_joint_.Calibrate();

    if (ps_.knee_sea_joint_.is_calibrated_ || !ps_.knee_sea_joint_.is_used_)
    {
        /** 已在knee_sea_joint_.Standby做了零力控制; 膝关节标定完成、进入零力控制后膝关节才走得动, 才能标定fsr */
        fsr_gait_estimator_.Calibrate();
    }

    ps_.is_calibrated_ =
        (ps_.fsr_gait_data_.is_calibrated_ || !ps_.fsr_gait_data_.is_used_) &&
        (ps_.hip_joint_.is_calibrated_ || !ps_.hip_joint_.is_used_) &&
        (ps_.knee_joint_.is_calibrated_ || !ps_.knee_joint_.is_used_) &&
        (ps_.ankle_joint_.is_calibrated_ || !ps_.ankle_joint_.is_used_) &&
        (ps_.knee_sea_joint_.is_calibrated_ || !ps_.knee_sea_joint_.is_used_);
}

void Side::Read()
{
    if (!ps_.is_used_) return;

    hip_joint_.Read();
    knee_joint_.Read();
    ankle_joint_.Read();

    knee_sea_joint_.Read();
}

bool Side::IsMotorConnect()
{
    if (!ps_.is_used_) return true;

    bool hip_ok = hip_joint_.IsMotorConnect();
    bool knee_ok = knee_joint_.IsMotorConnect();
    bool ankle_ok = ankle_joint_.IsMotorConnect();

    bool knee_sea_ok = knee_sea_joint_.IsMotorConnect();

    return hip_ok && knee_ok && ankle_ok && knee_sea_ok;
}

void Side::Standby()
{
    if (!ps_.is_used_) return;

    /** #TODO Standby的策略需要优化 */
    hip_joint_.Standby();
    knee_joint_.Standby();
    ankle_joint_.Standby();

    knee_sea_joint_.Standby();
}

void Side::Assist()
{
    if (!ps_.is_used_) return;

    hip_joint_.Assist();
    knee_joint_.Assist();
    ankle_joint_.Assist();

    knee_sea_joint_.Assist();
}

void Side::Shutdown()
{
    hip_joint_.Shutdown();
    knee_joint_.Shutdown();
    ankle_joint_.Shutdown();
    knee_sea_joint_.Shutdown();

    fsr_gait_estimator_.Reset();
}

void AdaptiveOscillator::Update()
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

    /**< 示教信号 */
    float left_angle_rad = pe_.left_side_.knee_joint_.pos_rad_;
    float right_angle_rad = pe_.right_side_.knee_joint_.pos_rad_;
    float left_velocity = pe_.left_side_.knee_joint_.vel_radps_;
    float right_velocity = pe_.right_side_.knee_joint_.vel_radps_;
    bool left_heel_contact_state = pe_.left_side_.fsr_gait_data_.heel_contact_state_;
    bool right_heel_contact_state = pe_.right_side_.fsr_gait_data_.heel_contact_state_;
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

    if (false) // just for debug
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
    else if (true) // just for debug
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
    if (true) // just for debug
    {
        if (delta_ts_s > 0.0f)
        {
            alpha = 1.0f - expf(- delta_ts_s / AdaptiveOscillator::kEmaTauS);
        }
        if ((pe_.ao_data_.left_event_cnt_ >= 1) != (pe_.ao_data_.right_event_cnt_ >= 1))
        {
            vel_energy_ema_ = vel_energy;
        }
        vel_energy_ema_ = alpha * vel_energy + (1.0f - alpha) * vel_energy_ema_;
        vel_energy_thresh = fmaxf(0.5f, 0.6f * vel_energy_ema_);
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
    if (true) // just for debug
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
    // bool is_stopping = is_stopping_low_energy || is_stopping_both_heel_contact; // 低能量太容易触发
    bool is_stopping = is_stopping_both_heel_contact;

    bool is_long_time_no_event = ((tnow_sys_us - left_tk_sys_us_ > AdaptiveOscillator::kMaxTstrideUs) && (pe_.ao_data_.left_event_cnt_ >= 1)) || ((tnow_sys_us - right_tk_sys_us_ > AdaptiveOscillator::kMaxTstrideUs) && (pe_.ao_data_.right_event_cnt_ >= 1));


    bool is_two_side_event_cnt_abnormal = (pe_.ao_data_.left_event_cnt_ > pe_.ao_data_.right_event_cnt_ + 1) || (pe_.ao_data_.right_event_cnt_ > pe_.ao_data_.left_event_cnt_ + 1);


    if (is_long_time_no_event || is_two_side_event_cnt_abnormal || is_stopping)
    {
        pe_.ao_data_.left_event_cnt_ = 0;
        pe_.ao_data_.right_event_cnt_ = 0;
        left_angle_pos_peak_prev = 0.45f;
        right_angle_pos_peak_prev = 0.45f;
    }

    bool is_left_event = false;
    bool is_right_event = false;
    if (false) // just for debug
    {
        is_left_event = is_left_angle_pos_peak;
        is_right_event = is_right_angle_pos_peak;
    }
    else if (true) // just for debug
    {
        is_left_event = is_left_heel_strike_event;
        is_right_event = is_right_heel_strike_event;
    }

    if (is_left_event)
    {
        if ((pe_.ao_data_.left_event_cnt_ < 1) || ((pe_.ao_data_.left_event_cnt_ >= 1) && (tnow_sys_us - left_tk_sys_us_ > kMinTstrideUs)))
        {
            pe_.ao_data_.left_event_cnt_ ++;
            left_tk_sys_us_ = tnow_sys_us;
            is_left_event = true;
        }
    }
    if (is_right_event)
    {
        if ((pe_.ao_data_.right_event_cnt_ < 1) || ((pe_.ao_data_.right_event_cnt_ >= 1) && (tnow_sys_us - right_tk_sys_us_ > kMinTstrideUs)))
        {
            pe_.ao_data_.right_event_cnt_ ++;
            right_tk_sys_us_ = tnow_sys_us;
            is_right_event = true;
        }
    }

    /**< 更新振荡器 */
    float dot_phi[kNumAOs] = {0};
    float dot_omega = 0;
    float dot_alpha[kNumAOs] = {0};
    float dot_alpha0 = 0;
    if (pe_.ao_data_.left_event_cnt_ >= 1 || pe_.ao_data_.right_event_cnt_ >= 1)
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

    pe_.ao_data_.left_phi_comp_rad_ = fmodf(phi_n + left_phi_e_, _2PI);
    if (pe_.ao_data_.left_phi_comp_rad_ < 0.0f) pe_.ao_data_.left_phi_comp_rad_ += _2PI;

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
    pe_.ao_data_.right_phi_comp_rad_ = fmodf(phi_n + right_phi_e_, _2PI);
    if (pe_.ao_data_.right_phi_comp_rad_ < 0.0f) pe_.ao_data_.right_phi_comp_rad_ += _2PI;

    tprev_sys_us_ = tnow_sys_us;
}

void AdaptiveOscillator::Reset()
{
    tprev_sys_us_ = 0;
    left_tk_sys_us_ = 0;
    right_tk_sys_us_ = 0;

    hat_x_ = 0.0f;
    omega_ = _2PI * 1.0f;
    arm_fill_f32(0.0f, phi_, kNumAOs);
    arm_fill_f32(0.2f, alpha_, kNumAOs);
    alpha0_ = 0.0f;
    vel_energy_ema_ = 0.0f;

    left_Pe_tilde_tk_ = 0.0f;
    right_Pe_tilde_tk_ = 0.0f;
    left_epsilon_phi_tk_ = 0.0f;
    right_epsilon_phi_tk_ = 0.0f;
    left_phi_e_ = 0.0f;
    right_phi_e_ = 0.0f;
}

ExoShell::ExoShell(UART_HandleTypeDef &huart, Exo &exo) : Shell(huart), exo_(exo)
{
    RegisterCommand("setled", CmdWrapper<ExoShell, &ExoShell::OnCmdSetLed>, this);
    RegisterCommand("setlocomode", CmdWrapper<ExoShell, &ExoShell::OnCmdSetLocoMode>, this);

    RegisterCommand("vofasw", [](void *ctx, int, char **) {
        auto& shell = *static_cast<ExoShell *>(ctx);
        shell.exo_.pe_.telemetry_config_.enable = ! shell.exo_.pe_.telemetry_config_.enable;
        if (shell.exo_.pe_.telemetry_config_.enable) shell.SendString("VOFA telemetry: ON\r\n");
        else shell.SendString("VOFA telemetry: OFF\r\n");
    }, this);

    RegisterCommand("estop", [](void *ctx, int, char **) {
        auto& shell = *static_cast<ExoShell *>(ctx);
        shell.exo_.pe_.pending_events_ |= ExoData::SysEvent::kEmergencyStop;
        shell.SendString("!!! EMERGENCY STOP !!!\r\n");
    }, this);

    RegisterCommand("wakeup", [](void *ctx, int, char **) {
        auto& shell = *static_cast<ExoShell *>(ctx);
        shell.exo_.pe_.pending_events_ |= ExoData::SysEvent::kWakeup;
        shell.SendString("-> Wakeup Requested\r\n");
    }, this);

    RegisterCommand("calib", [](void *ctx, int, char **) {
        auto& shell = *static_cast<ExoShell *>(ctx);
        shell.exo_.pe_.pending_events_ |= ExoData::SysEvent::kStartCalibrate;
        shell.SendString("-> Calibration Requested\r\n");
    }, this);

    RegisterCommand("start", [](void *ctx, int, char **) {
        auto& shell = *static_cast<ExoShell *>(ctx);
        shell.exo_.pe_.pending_events_ |= ExoData::SysEvent::kStartAssist;
        shell.SendString("-> Start Assist Requested\r\n");
    }, this);

    RegisterCommand("stop", [](void *ctx, int, char **) {
        auto& shell = *static_cast<ExoShell *>(ctx);
        shell.exo_.pe_.pending_events_ |= ExoData::SysEvent::kStopAssist;
        shell.SendString("-> Stop Assist Requested\r\n");
    }, this);

    RegisterCommand("sleep", [](void *ctx, int, char **) {
        auto& shell = *static_cast<ExoShell *>(ctx);
        shell.exo_.pe_.pending_events_ |= ExoData::SysEvent::kEnterSleep;
        shell.SendString("-> Sleep Requested\r\n");
    }, this);

    RegisterCommand("clearfaults", [](void *ctx, int, char **) {
        auto& shell = *static_cast<ExoShell *>(ctx);
        shell.exo_.pe_.pending_events_ |= ExoData::SysEvent::kClearFaults;
        shell.SendString("-> Clear Faults Requested\r\n");
    }, this);

    RegisterCommand("testsw", [](void *ctx, int, char **) {
        auto& shell = *static_cast<ExoShell *>(ctx);
        shell.exo_.pe_.do_test = ! shell.exo_.pe_.do_test;
        if (shell.exo_.pe_.do_test) shell.SendString("Test: ON\r\n");
        else shell.SendString("Test: OFF\r\n");
    }, this);

    /** 注册需要实时调节的参数 */
    RegisterRwParam("weight", &exo_.pe_.user_weight_kg_);
    RegisterRwParam("la_on",  &exo_.left_side_.ankle_joint_.assistance_start_phase_percent_);
    RegisterRwParam("la_off", &exo_.left_side_.ankle_joint_.assistance_end_phase_percent_);
    RegisterRwParam("la_pre", &exo_.left_side_.ankle_joint_.cable_pre_tensioned_position_);
    RegisterRwParam("la_ten", &exo_.left_side_.ankle_joint_.cable_tensioned_position_);
    // RegisterRwParam("poskp", &exo_.left_side_.knee_sea_joint_.motor_.pos_pid_.kp_);
    // RegisterRwParam("poski", &exo_.left_side_.knee_sea_joint_.motor_.pos_pid_.ki_);
    // RegisterRwParam("spdkp", &exo_.left_side_.knee_sea_joint_.motor_.speed_pid_.kp_);
    // RegisterRwParam("spdki", &exo_.left_side_.knee_sea_joint_.motor_.speed_pid_.ki_);
    // RegisterRwParam("poskp", &exo_.left_side_.knee_sea_joint_.joint_pos_pid_.kp_);
    // RegisterRwParam("poski", &exo_.left_side_.knee_sea_joint_.joint_pos_pid_.ki_);
    RegisterRwParam("forkp", &exo_.left_side_.knee_sea_joint_.spring_force_pid_.kp_);
    RegisterRwParam("forki", &exo_.left_side_.knee_sea_joint_.spring_force_pid_.ki_);
    RegisterRwParam("freq", &exo_.left_side_.knee_sea_joint_.force_test_sin_freq);
}

void ExoShell::OnCmdSetLed(int argc, char **argv)
{
    if (argc < 2 || argv == nullptr || argv[1] == nullptr)
    {
        SendString("Usage: setled <number>\r\n");
        return;
    }

    int state = GetInt(argc, argv, 1);
    exo_.state_led_.UpdateColorBDMA(state);
    SendString("LED state set!\r\n");
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
        exo_.pe_.override_usr_.enable_locomode_override = false;
        SendString("Mode Autonomous\r\n");
    }
    else if (strcmp(mode_str, "walking") == 0)
    {
        exo_.pe_.override_usr_.enable_locomode_override = true;
        exo_.pe_.override_usr_.forced_locomode = ExoData::LocoMode::kWalking;
        SendString("Mode set to: kWalking\r\n");
    }
    else if (strcmp(mode_str, "sit2stand") == 0)
    {
        exo_.pe_.override_usr_.enable_locomode_override = true;
        exo_.pe_.override_usr_.forced_locomode = ExoData::LocoMode::kSitToStand;
        SendString("Mode set to: kSitToStand\r\n");
    }
    else
    {
        Printf("Unknown mode: %s\r\n", mode_str);
    }
}

void Exo::Initialize()
{
    /** 调试: 重置标定标志. */
    ResetCalibrationFlags();

    /** 调试: 选择助力的关节 */
    pe_.left_side_.hip_joint_.is_used_ = false;
    pe_.right_side_.hip_joint_.is_used_ = false;
    pe_.left_side_.knee_joint_.is_used_ = false;
    pe_.right_side_.knee_joint_.is_used_ = false;
    pe_.left_side_.ankle_joint_.is_used_ = false;
    pe_.right_side_.ankle_joint_.is_used_ = false;

    pe_.left_side_.knee_sea_joint_.is_used_ = true;
    pe_.right_side_.knee_sea_joint_.is_used_ = false;

    /** 调试: 髋关节参数. */

    /** 调试: 膝关节参数. */

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

float duration_us = 0;  /** 一般来说少定义全局变量, 但这仅用于测试 */

void Exo::Run()
{
    uint32_t start_ticks = DWT_CYCCNT; /** 仅仅用于实际测试运行一次run()函数需要多长时间 */

    /** 读取/转换(有些传感器在中断回调中读取)传感器数据 */
    Read();    
    /** 在系统非睡眠状态下检查是否欠压 或 出现故障 */
    if (pe_.state_ != ExoData::State::kSleep)
    {
        CheckSystemHealth();   /** 重新计算error_code_ */
    }

    /** 力跟踪测试: 在此之前先标定好让位置去到零点, 然后记录 pos_linear_encoder_offset_mm_ */
    // static bool is_zeroed = false;
    // static uint8_t low_err_count = 0;
    // static float force_ref_N = 0;
    // if (pe_.do_test)
    // {
    //     if (!is_zeroed)
    //     {
    //         pe_.left_side_.knee_sea_joint_.pos_ref_rad_ = 0;
    //         left_side_.knee_sea_joint_.joint_pos_pid_.output_limit_ = left_side_.knee_sea_joint_.motor_.max_iqref_amp_ / 5.0f;
    //         left_side_.knee_sea_joint_.JointPosControl();

    //         if (pe_.left_side_.knee_sea_joint_.pos_rad_ > -0.01 && pe_.left_side_.knee_sea_joint_.pos_rad_ < 0.01)
    //         {
    //             low_err_count ++;
    //             if (low_err_count > 200)
    //             {
    //                 low_err_count = 0;
    //                 pe_.left_side_.knee_sea_joint_.pos_slider_offset_mm_ = pe_.left_side_.knee_sea_joint_.pos_slider_mm_;
    //                 left_side_.knee_sea_joint_.joint_pos_pid_.output_limit_ = left_side_.knee_sea_joint_.motor_.max_iqref_amp_;
    //                 is_zeroed = true;
    //             }
    //         }
    //         else
    //         {
    //             low_err_count = 0;
    //             is_zeroed = false;
    //         }
    //     }
    //     else
    //     {
    //         float sys_ms = (float)GetSysTimeMs();
    //         float freq = left_side_.knee_sea_joint_.force_test_sin_freq;
    //         float amp = 200.0f;
    //         float radi = _2PI * freq * sys_ms / 1000.0f;

    //         force_ref_N = arm_sin_f32(radi) * amp;
    //         pe_.left_side_.knee_sea_joint_.force_spring_ref_N_ = force_ref_N;
    //         left_side_.knee_sea_joint_.SpringForceControl();
    //     }
    // }
    // else
    // {
    //     if (!is_zeroed)
    //     {
    //         pe_.left_side_.knee_sea_joint_.force_spring_ref_N_ = force_ref_N;
    //         left_side_.knee_sea_joint_.motor_.DisableMotor();
    //     }
    //     else
    //     {
    //         left_side_.knee_sea_joint_.Standby();
    //     }
    // }

    /** 根据系统当前状态过滤无效事件, 比如在kSleep状态下只接受wakeup命令 */
    pe_.pending_events_ &= AllowedEventsForState(pe_.state_);

    /** 用户发起了estop急停命令 */
    const bool is_estop_triggered = ((pe_.pending_events_ & ExoData::SysEvent::kEmergencyStop) != 0);
    if (is_estop_triggered)
    {
        ClearNonCriticalEvents(pe_);
        pe_.state_ = ExoData::State::kSleep;
        Shutdown();
        return;      /** 不清除estop事件标志, 不再运行下面的代码 */
    }

    /** 电池欠压则将状态机转入kFaultLowBattery */
    const bool battery_low = ((pe_.error_code_ & ExoData::Error::kBatteryLow) != 0);
    const bool has_any_fault = (pe_.error_code_ != ExoData::Error::kNone);
    if (battery_low)
    {
        if (pe_.state_ != ExoData::State::kFaultLowBattery)
        {
            ClearNonCriticalEvents(pe_);
            pe_.state_ = ExoData::State::kFaultLowBattery;
        }
    }
    /** 出现故障则将状态机转入kFaultSystem */
    else if (has_any_fault)
    {
        if (pe_.state_ != ExoData::State::kFaultSystem)
        {
            ClearNonCriticalEvents(pe_);
            pe_.state_ = ExoData::State::kFaultSystem;
        }
    }
    /** 用户的休眠命令可令状态机直接转入kSleep */
    else if ((pe_.pending_events_ & ExoData::SysEvent::kEnterSleep) != 0)
    {
        pe_.pending_events_ &= ~ExoData::SysEvent::kEnterSleep;
        pe_.state_ = ExoData::State::kSleep;
        Shutdown();
    }

    /** 外骨骼顶层状态机, 系统休眠-运行-报错... */
    switch (pe_.state_)
    {
    case ExoData::State::kSleep:
        /** 收到wakeup命令并且电压足够则转入kWaitMotorComm */
        if (((pe_.pending_events_ & ExoData::SysEvent::kWakeup) != 0) && pe_.battery_voltage_ >= 19.5f)
        {
            pe_.pending_events_ &= ~ExoData::SysEvent::kWakeup;
            pe_.state_ = ExoData::State::kWaitMotorComm;
        }
        break;

    case ExoData::State::kWaitMotorComm:
        /** 接收到calib命令并且电机通信检查完毕则转入kCalibrating */
        if (((pe_.pending_events_ & ExoData::SysEvent::kStartCalibrate) != 0) && IsMotorConnect())
        {
            pe_.pending_events_ &= ~ExoData::SysEvent::kStartCalibrate;
            ResetCalibrationFlags();
            pe_.state_ = ExoData::State::kCalibrating;
        }
        break;

    case ExoData::State::kCalibrating:
        Calibrate();
        Standby();   /** 为了获取电机/关节状态, 保持通信; 膝sea零力控制 */
        /** 标定完毕则转入kReady */
        if (IsCalibrateDone())
        {
            pe_.state_ = ExoData::State::kReady;
        }
        break;

    case ExoData::State::kReady:
        Estimate();
        Standby();   /** 为了获取电机/关节状态, 保持通信 */
        /** 此时如果对标定结果不满意可发起calib命令重新标定 */
        if ((pe_.pending_events_ & ExoData::SysEvent::kStartCalibrate) != 0)
        {
            pe_.pending_events_ &= ~ExoData::SysEvent::kStartCalibrate;
            ResetCalibrationFlags();
            pe_.state_ = ExoData::State::kCalibrating;
        }
        /** 如果用户发起了start命令则转入kAssisting */
        else if ((pe_.pending_events_ & ExoData::SysEvent::kStartAssist) != 0)
        {
            pe_.pending_events_ &= ~ExoData::SysEvent::kStartAssist;
            pe_.state_ = ExoData::State::kAssisting;
        }
        break;

    case ExoData::State::kAssisting:
        /** 估计运动模式及该模式下的参数, 如walking及步态相位 */
        Estimate();
        /** 如果用户发起了stop命令则回到kReady */
        if ((pe_.pending_events_ & ExoData::SysEvent::kStopAssist) != 0)
        {
            pe_.pending_events_ &= ~ExoData::SysEvent::kStopAssist;
            Standby();
            pe_.state_ = ExoData::State::kReady;
        }
        /** 自动检测停止辅助的条件, 暂未实现 */  
        // else if (IsStaticMotionIntent())     
        // {
        //     Standby();
        //     pe_.state_ = ExoData::State::kReady;
        // }
        /** 进行助力 */
        else
        {
            Assist();
        }
        break;

    case ExoData::State::kFaultLowBattery:
        Shutdown();
        /** 不关机充电(?不知合理否)到大于19.5V则重转入kSleep */  
        if (pe_.battery_voltage_ >= 19.5f)
        {
            pe_.state_ = ExoData::State::kSleep;
        }
        break;

    case ExoData::State::kFaultSystem:
        Shutdown();
        /** 最好不用ClearFaults命令, 有问题就关机排查 */  
        if (((pe_.pending_events_ & ExoData::SysEvent::kClearFaults) != 0))
        {
            pe_.pending_events_ &= ~ExoData::SysEvent::kClearFaults;
            pe_.error_code_ = ExoData::Error::kNone;
            pe_.state_ = ExoData::State::kSleep;
        }
        break;

    default:
        break;
    }
    dji_esc_hub_.SendAllCanTxData(); /** important */


    /** 处理上位机蓝牙发下来的命令, 所有命令见ExoShell的构造函数 */  
    if (shell_.ProcessPendingCommand())
    {
        pe_.telemetry_config_.pause_until_ms = GetSysTimeMs() + 3000U;
    }

    /** 由于处理命令后需要反馈给上位机, 为了反馈不被数据掩盖, 延时一段时间再发数据 */  
    const uint32_t now_ms = GetSysTimeMs();
    // if (pe_.telemetry_config_.enable && (now_ms >= pe_.telemetry_config_.pause_until_ms))
    // {
        VofaSendTelemetry();
    // }

    /** 指示系统状态机当前是什么状态 */
    state_led_.UpdateColorBDMA(static_cast<uint8_t>(pe_.state_));

    /** 计算得到运行一次run()函数需要用到的时间 */
    duration_us = DWTGetDeltaUs(start_ticks);
}

void Exo::Calibrate()
{
    left_side_.Calibrate();
    right_side_.Calibrate();
}

void Exo::ResetCalibrationFlags()
{
    pe_.left_side_.is_calibrated_ = false;
    pe_.left_side_.hip_joint_.is_calibrated_ = true;  /** 暂不需要标定 */
    pe_.left_side_.knee_joint_.is_calibrated_ = true;
    pe_.left_side_.ankle_joint_.is_calibrated_ = true;
    pe_.left_side_.knee_sea_joint_.is_calibrated_ = false; /** 需要标定 */
    pe_.left_side_.fsr_gait_data_.is_calibrated_ = false;
    pe_.left_side_.fsr_gait_data_.do_calibration_heel_fsr_ = true;
    pe_.left_side_.fsr_gait_data_.do_calibration_toe_fsr_ = false;
    pe_.left_side_.fsr_gait_data_.do_calibration_refinement_heel_fsr_ = true;
    pe_.left_side_.fsr_gait_data_.do_calibration_refinement_toe_fsr_ = false;

    pe_.right_side_.is_calibrated_ = false;
    pe_.right_side_.hip_joint_.is_calibrated_ = true; /** 暂不需要标定 */
    pe_.right_side_.knee_joint_.is_calibrated_ = true;
    pe_.right_side_.ankle_joint_.is_calibrated_ = true;
    pe_.right_side_.knee_sea_joint_.is_calibrated_ = false; /** 需要标定 */
    pe_.right_side_.fsr_gait_data_.is_calibrated_ = false;
    pe_.right_side_.fsr_gait_data_.do_calibration_heel_fsr_ = true;
    pe_.right_side_.fsr_gait_data_.do_calibration_toe_fsr_ = false;
    pe_.right_side_.fsr_gait_data_.do_calibration_refinement_heel_fsr_ = true;
    pe_.right_side_.fsr_gait_data_.do_calibration_refinement_toe_fsr_ = false;
}

void Exo::Read()
{
    pe_.battery_voltage_ = (g_adc_data[0] * 3.3f / 65535) * 11;
    pe_.battery_voltage_ = 24; /** #HACK 强制令电压读数大于唤醒电压 */
    left_side_.Read();
    right_side_.Read();
}

void Exo::Estimate()
{
    /** high level control */
    // if (pe_.override_usr_.enable_locomode_override)
    if (true)  /** #HACK: 目前先使用用户选择的运动模式 */
    {
        pe_.loco_mode_ = pe_.override_usr_.forced_locomode;
    }
    else
    {
        /** #TODO: 在此执行自动运动意图检测, 并得到pe_.loco_mode_ */
    }

    /** Mid level control */
    if (pe_.loco_mode_ == ExoData::LocoMode::kWalking)
    {
        /**  */
        left_side_.fsr_gait_estimator_.Update();
        right_side_.fsr_gait_estimator_.Update();
        ao_.Update();
    }
    else
    {
        /** #TODO: 进行其他活动的中层控制; 如检测坐立转换时刻 */
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
    pe_.error_code_ = ExoData::Error::kNone;

    if (pe_.battery_voltage_ < 19.0f)
    {
        pe_.error_code_ |= ExoData::Error::kBatteryLow;
    }
    if (left_side_.knee_joint_.motor_.error_code_ != 0 || left_side_.knee_joint_.motor_.fault_code_ != 0)
    {
        pe_.error_code_ |= ExoData::Error::kLeftKneeFault;
    }
    if (right_side_.knee_joint_.motor_.error_code_ != 0 || right_side_.knee_joint_.motor_.fault_code_ != 0)
    {
        pe_.error_code_ |= ExoData::Error::kRightKneeFault;
    }
    if (left_side_.ankle_joint_.motor_.error_code_ != 0 || left_side_.ankle_joint_.motor_.fault_code_ != 0)
    {
        pe_.error_code_ |= ExoData::Error::kLeftAnkleFault;
    }
    if (right_side_.ankle_joint_.motor_.error_code_ != 0 || right_side_.ankle_joint_.motor_.fault_code_ != 0)
    {
        pe_.error_code_ |= ExoData::Error::kRightAnkleFault;
    }
}

#include "usbd_cdc_if.h"
void Exo::VofaSendTelemetry()
{
    static uint32_t loop_cnt = 0;
    // shell_.SetVofaJustFloatData(0, loop_cnt++);
    // shell_.SetVofaJustFloatData(1, pe_.left_side_.fsr_gait_data_.heel_.raw_reading);
    // shell_.SetVofaJustFloatData(2, pe_.left_side_.fsr_gait_data_.toe_.raw_reading);
    // shell_.SetVofaJustFloatData(3, pe_.right_side_.fsr_gait_data_.heel_.raw_reading);
    // shell_.SetVofaJustFloatData(4, pe_.right_side_.fsr_gait_data_.toe_.raw_reading);
    // shell_.SetVofaJustFloatData(5, pe_.left_side_.fsr_gait_data_.percent_gait_ / 100.0f);
    // shell_.SetVofaJustFloatData(6, pe_.right_side_.fsr_gait_data_.percent_gait_ / 100.0f);
    // shell_.SetVofaJustFloatData(7, left_side_.ankle_joint_.motor_.position_ref_);
    // shell_.SetVofaJustFloatData(8, right_side_.ankle_joint_.motor_.position_ref_);
    // shell_.SetVofaJustFloatData(9, left_side_.ankle_joint_.motor_.position_);
    // shell_.SetVofaJustFloatData(10, right_side_.ankle_joint_.motor_.position_);
    // shell_.SetVofaJustFloatData(11, pe_.left_side_.ankle_joint_.plantarflexion_force_N_);
    // shell_.SetVofaJustFloatData(12, pe_.right_side_.ankle_joint_.plantarflexion_force_N_);

    // shell_.SetVofaJustFloatData(10, pe_.left_side_.knee_sea_joint_.pos_linear_encoder_mm_);
    // shell_.SetVofaJustFloatData(11, left_side_.knee_sea_joint_.motor_.shaft_pos_reference_rad_);
    // shell_.SetVofaJustFloatData(12, left_side_.knee_sea_joint_.motor_.shaft_pos_feedback_rad_);
    // shell_.SendVofaJustFloatFrame(13);

    DmaBuffer buf = {0};
    buf.f_data[0] = loop_cnt++;
    buf.f_data[1] = pe_.left_side_.fsr_gait_data_.heel_.raw_reading;
    // buf.f_data[2] = pe_.left_side_.fsr_gait_data_.toe_.raw_reading;
    buf.f_data[2] = pe_.left_side_.fsr_gait_data_.percent_gait_ / 100.0f;
    buf.f_data[3] = pe_.right_side_.fsr_gait_data_.heel_.raw_reading;
    // buf.f_data[4] = pe_.right_side_.fsr_gait_data_.toe_.raw_reading;
    buf.f_data[4] = pe_.right_side_.fsr_gait_data_.percent_gait_ / 100.0f;
    // buf.f_data[5] = pe_.left_side_.fsr_gait_data_.percent_gait_ / 100.0f;
    // buf.f_data[6] = pe_.right_side_.fsr_gait_data_.percent_gait_ / 100.0f;
    // buf.f_data[7] = left_side_.ankle_joint_.motor_.position_ref_;
    // buf.f_data[8] = right_side_.ankle_joint_.motor_.position_ref_;
    // buf.f_data[9] = left_side_.ankle_joint_.motor_.position_;

    // buf.f_data[5] = pe_.left_side_.knee_sea_joint_.pos_linear_encoder_mm_;
    buf.f_data[5] = force_ref_temp;
    buf.f_data[6] = left_side_.knee_sea_joint_.motor_.rotor_iq_reference_amp_;
    buf.f_data[7] = pe_.left_side_.knee_sea_joint_.force_spring_ref_N_;
    buf.f_data[8] = pe_.left_side_.knee_sea_joint_.force_spring_N_;
    buf.f_data[9] = pe_.left_side_.knee_sea_joint_.pos_ref_rad_;
    buf.f_data[10] = pe_.left_side_.knee_sea_joint_.pos_rad_;

    buf.f_data[11] = left_side_.knee_sea_joint_.motor_.shaft_pos_reference_rad_;
    buf.f_data[12] = left_side_.knee_sea_joint_.motor_.shaft_pos_feedback_rad_;
    buf.f_data[13] = left_side_.knee_sea_joint_.motor_.shaft_speed_reference_radps_;
    buf.f_data[14] = left_side_.knee_sea_joint_.motor_.shaft_speed_feedback_radps_;
    buf.f_data[15] = duration_us;

    uint16_t count = 4 * 16; /** 4 x 浮点数个数 */
    buf.u8_data[count++] = 0x00;
    buf.u8_data[count++] = 0x00;
    buf.u8_data[count++] = 0x80;
    buf.u8_data[count++] = 0x7f;
    CDC_Transmit_HS(buf.u8_data, count);
}

bool Exo::IsMotorConnect()
{
    return left_side_.IsMotorConnect() && right_side_.IsMotorConnect();
}

bool Exo::IsCalibrateDone()
{
    return pe_.left_side_.is_calibrated_ && pe_.right_side_.is_calibrated_;
}

bool Exo::IsStopWalking()
{
    return pe_.ao_data_.left_event_cnt_ <= 1 && pe_.ao_data_.right_event_cnt_ <= 1;
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

    case ExoData::State::kFaultLowBattery:
        break;

    case ExoData::State::kFaultSystem:
        allowed_events |= ExoData::SysEvent::kClearFaults;
        break;

    default:
        break;
    }
    return allowed_events;
}

void Exo::SensorUartRxCallback(const uint8_t *data, uint16_t data_size)
{
    if (data_size == sizeof(exo_sensor_packet_t))
    {
        exo_sensor_packet_t *packet = (exo_sensor_packet_t *)data;
        left_side_.ps_.fsr_gait_data_.heel_.raw_reading = 3.4f - packet->left_foot.mV_heel / 1000.0f;
        left_side_.ps_.fsr_gait_data_.toe_.raw_reading = 3.4f - packet->left_foot.mV_toe / 1000.0f;
        left_side_.ps_.ankle_joint_.plantarflexion_force_N_ = packet->left_foot.mV_pull;
        left_side_.ps_.foot_imu_.quat_i_ = packet->left_foot.quatI;
        left_side_.ps_.foot_imu_.quat_j_ = packet->left_foot.quatJ;
        left_side_.ps_.foot_imu_.quat_k_ = packet->left_foot.quatK;
        left_side_.ps_.foot_imu_.quat_real_ = packet->left_foot.quatReal;
        
        right_side_.ps_.fsr_gait_data_.heel_.raw_reading = 3.4f - packet->right_foot.mV_heel / 1000.0f;
        right_side_.ps_.fsr_gait_data_.toe_.raw_reading = 3.4f - packet->right_foot.mV_toe / 1000.0f;
        right_side_.ps_.ankle_joint_.plantarflexion_force_N_ = packet->right_foot.mV_pull;
        right_side_.ps_.foot_imu_.quat_i_ = packet->right_foot.quatI;
        right_side_.ps_.foot_imu_.quat_j_ = packet->right_foot.quatJ;
        right_side_.ps_.foot_imu_.quat_k_ = packet->right_foot.quatK;
        right_side_.ps_.foot_imu_.quat_real_ = packet->right_foot.quatReal;
    }
}

void Exo::UsrShellUartRxCallback(const uint8_t *data, uint16_t data_size)
{
    shell_.PushPendingCommand(data, data_size);
}


void Exo::CanRxCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t can_id, const uint8_t *data)
{
    if (hfdcan != &hw_.motor_can) return;

    /** 默认下面全部电机都使用fdcan1 */
    left_side_.hip_joint_.motor_.CanRxCallBack(can_id, data);
    right_side_.hip_joint_.motor_.CanRxCallBack(can_id, data);
    left_side_.knee_joint_.motor_.CanRxCallBack(can_id, data);
    right_side_.knee_joint_.motor_.CanRxCallBack(can_id, data);
    left_side_.ankle_joint_.motor_.CanRxCallBack(can_id, data);
    right_side_.ankle_joint_.motor_.CanRxCallBack(can_id, data);
    dji_esc_hub_.CanRxCallBack(can_id, data); 
}

void Exo::UartRxCallback(UART_HandleTypeDef *huart, uint16_t data_size)
{
    const uint8_t *data = huart->pRxBuffPtr;
    if (huart == &hw_.sensor_uart)
    {
        SensorUartRxCallback(data, data_size);

    }
    else if (huart == &hw_.shell_uart)
    {
        UsrShellUartRxCallback(data, data_size);
    }
    else /** 暂时由底层函数判断及重启DMA接受 */
    {
        left_side_.knee_sea_joint_.mag_encoder_.UartRxCallback(huart, data, data_size);
        right_side_.knee_sea_joint_.mag_encoder_.UartRxCallback(huart, data, data_size);
    }
}

/* ------------------ C wrapper ------------------- */
void CallExoCanRxCallBack(Exo *exo, FDCAN_HandleTypeDef *hfdcan, uint32_t can_ext_id, const uint8_t *rx_data)
{
    if (exo == nullptr || rx_data == nullptr) return;

    exo->CanRxCallback(hfdcan, can_ext_id, rx_data);
}

void CallExoUartRxCallback(Exo *exo, UART_HandleTypeDef *huart, uint16_t data_size)
{
    if (exo == nullptr || huart == nullptr) return;

    exo->UartRxCallback(huart, data_size);
}