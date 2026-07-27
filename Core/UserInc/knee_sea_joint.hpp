#ifndef KNEE_SEA_JOINT_HPP
#define KNEE_SEA_JOINT_HPP

#include <cstdint>
#include "dji_esc.hpp"
#include "mag_encoder.hpp"
#include "pid.hpp"
#include "force_profile_generator.hpp"

class ExoData;
class SideData;
class KneeSeaJointData;

class KneeSeaJoint
{
public:
  enum class CtrlMode : uint8_t
  {
    kPosition,
    kSpringForce,
  };

  KneeSeaJoint(ExoData &pe,
               SideData &side_data,
               KneeSeaJointData &joint_data,
               DjiEscHub &dji_esc_hub,
               DjiEsc::EscId esc_id,
               UART_HandleTypeDef &huart) :
    pe_(pe),
    ps_(side_data),
    pj_(joint_data),
    motor_(dji_esc_hub, esc_id),
    mag_encoder_(huart),
    joint_pos_pid_(5.0f, 1.0f, 0.0f, -100.0f, motor_.max_iqref_amp_),
    spring_force_pid_(1.0f, 0.1f, 0.0f, -100.0f, motor_.max_iqref_amp_)
  {
  }
  virtual ~KneeSeaJoint() = default;

  void Calibrate();
  void Read();
  bool IsMotorConnect();
  void Shutdown();
  void Standby();
  void Assist();

  void JointPosControl();
  void SpringForceControl();

  ExoData &pe_;
  SideData &ps_;
  KneeSeaJointData &pj_;
  DjiEsc motor_;
  MagEncoder mag_encoder_;
  PIDController joint_pos_pid_;
  PIDController spring_force_pid_;
  KneeForceProfileGenerator force_profile_generator_;

private:
  CtrlMode ctrl_mode_ = CtrlMode::kSpringForce;
};

#endif
