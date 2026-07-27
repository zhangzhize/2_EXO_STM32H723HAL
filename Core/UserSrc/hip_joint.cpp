#include "exo.hpp"

void HipJoint::Calibrate()
{
  if (pj_.is_actuator_enabled_ && !pj_.is_link_pos_offset_valid_)
  {
    pj_.link_pos_offset_rad_ = pj_.link_pos_rad_;
    pj_.is_link_pos_offset_valid_ = true;
  }

  if (pe_.body_imu_.IsUsable() && ps_.thigh_imu_.IsUsable() && !pj_.is_sagittal_pos_offset_valid_)
  {
    pj_.sagittal_pos_offset_rad_ =
      WrapPi(ps_.thigh_imu_.SagittalRawRad() - pe_.body_imu_.SagittalRawRad());
    pj_.is_sagittal_pos_offset_valid_ = true;
  }
}

void HipJoint::Read()
{
  if (!pj_.is_actuator_enabled_) return;

  if (pj_.is_left_)
  {
    pj_.link_pos_rad_ = motor_.feedback_.pos_rad_ - pj_.link_pos_offset_rad_;
    pj_.UpdateLinkVelocity(motor_.feedback_.vel_radps_);
    pj_.tor_output_Nm_ = motor_.feedback_.tor_output_Nm_;
  }
  else
  {
    pj_.link_pos_rad_ = -motor_.feedback_.pos_rad_ - pj_.link_pos_offset_rad_;
    pj_.UpdateLinkVelocity(-motor_.feedback_.vel_radps_);
    pj_.tor_output_Nm_ = -motor_.feedback_.tor_output_Nm_;
  }
}

bool HipJoint::IsMotorConnect()
{
  if (!pj_.is_actuator_enabled_) return true;

  motor_.EnableMotor();
  return motor_.feedback_.flag_ > 0;
}

void HipJoint::Shutdown()
{
  motor_.DisableMotor();
}

void HipJoint::Standby()
{
  if (!pj_.is_actuator_enabled_) return;

  motor_.ctrl_param_.mode_id_ = DMMotorModeID::kMIT;
  motor_.ctrl_param_.kp_set_ = 0.0f;
  motor_.ctrl_param_.kd_set_ = 0.0f;
  motor_.ctrl_param_.pos_set_rad_ = 0.0f;
  motor_.ctrl_param_.vel_set_radps_ = 0.0f;
  motor_.ctrl_param_.tor_set_Nm_ = 0.0f;
  motor_.MitControl();
}

namespace
{
struct HipFFShared
{
  static constexpr int kHistSize = 128;
  static constexpr int kDelaySamples = 35;
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

    const float tau = kK * (arm_sin_f32(posR[past]) - arm_sin_f32(posL[past]));
    tau_left = tau;
    tau_right = tau;
    idx = (idx + 1) % kHistSize;
  }
};

HipFFShared g_hip_ff;
}

void HipJoint::Assist()
{
  if (!pj_.is_actuator_enabled_) return;

  const float posL_now = pe_.left_side_.hip_joint_.sagittal_pos_rad_;
  const float posR_now = pe_.right_side_.hip_joint_.sagittal_pos_rad_;
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
}
