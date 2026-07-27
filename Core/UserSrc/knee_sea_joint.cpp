#include "exo.hpp"

void KneeSeaJoint::Calibrate()
{
  if (pj_.is_actuator_enabled_ && !pj_.is_link_pos_offset_valid_)
  {
    if (pj_.pos_linear_encoder_mm_ < 0) return;
    pj_.pos_slider_offset_mm_ = pj_.pos_slider_mm_ - pj_.pos_linear_encoder_mm_;
    pj_.is_link_pos_offset_valid_ = true;
  }
}

void KneeSeaJoint::Read()
{
  if (!pj_.is_actuator_enabled_) return;

  pj_.pos_slider_mm_ = pj_.screw_lead_rad2mm_ * motor_.shaft_pos_feedback_rad_ - pj_.pos_slider_offset_mm_;
  pj_.vel_slider_mmps_ = pj_.screw_lead_rad2mm_ * motor_.shaft_speed_feedback_radps_;

  pj_.pos_linear_encoder_mm_ = mag_encoder_.absolute_position_mm_ - pj_.pos_linear_encoder_offset_mm_;
  pj_.vel_linear_encoder_mmps_ = 0.0f;

  pj_.pos_bias_mm_ = pj_.pos_slider_mm_ - pj_.pos_linear_encoder_mm_;
  pj_.force_spring_N_ = pj_.spring_stiffness_Npmm_ * pj_.pos_bias_mm_;

  pj_.link_pos_rad_ = pj_.pos_linear_encoder_mm_ /
                      (pj_.max_pos_linear_encoder_mm_ - pj_.pos_linear_encoder_offset_mm_) *
                      _PI_2;
  pj_.link_vel_radps_ = 0.0f;

  mag_encoder_.SendRequest();
}

bool KneeSeaJoint::IsMotorConnect()
{
  if (!pj_.is_actuator_enabled_) return true;
  return true;
}

void KneeSeaJoint::Shutdown()
{
  motor_.DisableMotor();
}

void KneeSeaJoint::Standby()
{
  if (!pj_.is_actuator_enabled_ || !pj_.is_link_pos_offset_valid_) return;

  const float force_profile = 0.0f;
  pj_.force_spring_ref_N_ = force_profile * pe_.user_info_.weight_kg_;
  SpringForceControl();
}

void KneeSeaJoint::Assist()
{
  if (!pj_.is_actuator_enabled_) return;

  float force_profile = 0.0f;
  const FsrGaitData &fsr = pj_.is_left_ ?
                               pe_.left_side_.fsr_gait_data_ :
                               pe_.right_side_.fsr_gait_data_;

  if (fsr.is_phase_valid_)
  {
    force_profile = force_profile_generator_.GetForceProfile(
        fsr.percent_gait_,
        pj_.sagittal_pos_rad_,
        pj_.sagittal_vel_radps_);
  }

  pj_.force_spring_ref_N_ = force_profile * pe_.user_info_.weight_kg_;
  SpringForceControl();
}

void KneeSeaJoint::JointPosControl()
{
  if (ctrl_mode_ != CtrlMode::kPosition)
  {
    joint_pos_pid_.ResetError();
    spring_force_pid_.ResetError();
    ctrl_mode_ = CtrlMode::kPosition;
  }

  const float pos_err_rad = pj_.link_pos_ctrlref_rad_ - pj_.link_pos_rad_;
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

  const float force_err_N = pj_.force_spring_ref_N_ - pj_.force_spring_N_;
  motor_.rotor_iq_reference_amp_ = spring_force_pid_(force_err_N);
  motor_.CurrentControl();
}
