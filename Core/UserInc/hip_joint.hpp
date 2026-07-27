#ifndef HIP_JOINT_HPP
#define HIP_JOINT_HPP

#include "dm_motor.hpp"

class ExoData;
class SideData;
class JointData;

/**
 * @brief 髋关节控制器 (达妙 DM4340 电机, MIT 模式)
 * @note  助力策略使用左右髋关节耦合前馈。
 */
class HipJoint
{
public:
  HipJoint(ExoData &pe,
           SideData &side_data,
           JointData &joint_data,
           FDCAN_HandleTypeDef &motor_can,
           uint16_t motor_can_id) :
    pe_(pe),
    ps_(side_data),
    pj_(joint_data),
    motor_(motor_can, motor_can_id)
  {
  }
  virtual ~HipJoint() = default;

  void Calibrate();
  void Read();
  bool IsMotorConnect();
  void Shutdown();
  void Standby();
  void Assist();

  ExoData &pe_;
  SideData &ps_;
  JointData &pj_;
  DMMotor motor_;
};

#endif
