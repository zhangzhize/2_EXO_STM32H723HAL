/**
 ******************************************************************************
 * @file    exo.hpp
 * @author  zzz
 * @brief   外骨骼系统头文件 —— 数据结构、关节类、状态机、辅助模块的声明
 * @version 0.1
 * @date    2025-12-15
 *
 * @copyright Copyright (c) 2025
 ******************************************************************************
 */
#ifndef EXO_HPP
#define EXO_HPP

#include <cstdint>
#include <math.h>
#include "utils.h"
extern "C"
{
#include "arm_math.h"
}
#include "status_led.hpp"
#include "robstride.hpp"
#include "dji_esc.hpp"
#include "force_profile_generator.hpp"
#include "pid.hpp"
#include "disturbance_observer.hpp"
#include "shell.hpp"
#include "chirp_generator.hpp"
#include "mahony.hpp"
#include "hip_joint.hpp"
#include "knee_sea_joint.hpp"

typedef struct __attribute__((packed)) foot_sensor_packet_v2_t
{
  // float force_heel_N;
  // float force_toe_N;
  float heel_raw_reading;
  float toe_raw_reading;
  float cop_x_mm;
  float cop_y_mm;
  float quatI;
  float quatJ;
  float quatK;
  float quatReal;
} foot_sensor_packet_v2_t;

typedef struct __attribute__((packed)) exo_sensor_packet_v2_t
{
  foot_sensor_packet_v2_t left_foot; /*!< 左足传感器数据 */
  foot_sensor_packet_v2_t right_foot; /*!< 右足传感器数据 */
} exo_sensor_packet_v2_t;

/**
 * @brief 为 enum class 生成位运算操作符 (|, &, ^, ~)
 * @note  用于 Error 和 SysEvent 位掩码枚举, 使其支持 flags |= kXxx 语法
 */
/* clang-format off */
#include <type_traits>
#define DEFINE_ENUM_CLASS_BITWISE_OPS(EnumType) \
    inline constexpr EnumType operator|(EnumType a, EnumType b) { \
        return static_cast<EnumType>(static_cast<std::underlying_type_t<EnumType>>(a) | static_cast<std::underlying_type_t<EnumType>>(b)); \
    } \
    inline constexpr EnumType& operator|=(EnumType& a, EnumType b) { \
        a = a | b; \
        return a; \
    } \
    inline constexpr EnumType operator&(EnumType a, EnumType b) { \
        return static_cast<EnumType>(static_cast<std::underlying_type_t<EnumType>>(a) & static_cast<std::underlying_type_t<EnumType>>(b)); \
    } \
    inline constexpr EnumType& operator&=(EnumType& a, EnumType b) { \
        a = a & b; \
        return a; \
    } \
    inline constexpr EnumType operator^(EnumType a, EnumType b) { \
        return static_cast<EnumType>(static_cast<std::underlying_type_t<EnumType>>(a) ^ static_cast<std::underlying_type_t<EnumType>>(b)); \
    } \
    inline constexpr EnumType& operator^=(EnumType& a, EnumType b) { \
        a = a ^ b; \
        return a; \
    } \
    inline constexpr EnumType operator~(EnumType a) { \
        return static_cast<EnumType>(~static_cast<std::underlying_type_t<EnumType>>(a)); \
    }
/* clang-format on */

/* 类前向声明 */
class ImuData;
class JointData;
class AnkleData;
class KneeSeaJointData;
class FsrGaitData;
class StairPhaseData;
class StsPhaseData;
class AoData;
class SideData;
class IntentionData;
class ExoData;

class AnkleJoint;
class KneeJoint;
class FsrGaitEstimator;
class StairPhaseEstimator;
class StsPhaseEstimator;
class AdaptiveOscillator;
class Side;
class ExoShell;
class BodyImu;
class DaMiaoImuHub;
class HiPnucImuHub;
class StreamingFeatureExtractor;
class IntentionRecognizer;
class Exo;

/**
 * @brief IMU 数据类
 * @note 1. sagittal_raw_rad_ 表示肢段相对大地的矢状面角度,
 *          由所选欧拉角或四元数重力投影计算.
 *       2. sagittal_pos_rad_ 表示相对站立参考姿态的矢状面角度;
 *          仅当 is_stand_posture_valid_ 为 true 时, 0 表示与站立参考一致.
 *          未捕获站立参考时, sagittal_pos_rad_ 保持为 0;
 *          未参考角度应显式读取 sagittal_raw_rad_.
 *       3. sagittal_gyro_radps_ 及其低通值表示对应矢状轴角速度,
 *          不减去站立参考.
 *       4. 期望正方向为: 躯干前倾, 大腿前摆, 小腿前摆, 足尖向上;
 *          实际方向由安装方式, sagittal_source_ 和 sagittal_direction_sign_ 共同决定.
 */
class ImuData
{
public:
  enum class SagittalSource : uint8_t
  {
    kEulerPitch,
    kEulerRoll,
    kGravityXz,
    kGravityYz,
  };

  explicit ImuData(bool is_left = true,
                   float sagittal_direction_sign = 1.0f,
                   SagittalSource sagittal_source = SagittalSource::kEulerPitch,
                   float sample_rate_hz = 200.0f) :
    is_left_(is_left),
    sagittal_direction_sign_(sagittal_direction_sign >= 0.0f ? 1.0f : -1.0f),
    sagittal_source_(sagittal_source),
    sagittal_gyro_lpf_alpha_(ComputeLpfAlpha(sample_rate_hz))
  {
  }
  virtual ~ImuData() = default;

  float q_[4] = {1.0f, 0.0f, 0.0f, 0.0f}; /*!< 四元数, 顺序: real, i, j, k */
  float roll_rad_ = 0.0f;
  float pitch_rad_ = 0.0f;
  float yaw_rad_ = 0.0f;

  float sagittal_raw_rad_ = 0.0f;
  float sagittal_pos_rad_ = 0.0f;
  float sagittal_stand_ref_rad_ = 0.0f;
  float sagittal_gyro_radps_ = 0.0f;
  float sagittal_gyro_lpf_radps_ = 0.0f;
  bool is_stand_posture_valid_ = false;

  float roll_stand_rad_ = 0.0f;
  float pitch_stand_rad_ = 0.0f;
  float yaw_stand_rad_ = 0.0f;

  /* 传感器原始数据 */
  float accel_mps2_[3] = {0.0f, 0.0f, 0.0f}; /*!< 传感器坐标顺序: x, y, z */
  float gyro_radps_[3] = {0.0f, 0.0f, 0.0f}; /*!< 传感器坐标顺序: x, y, z */
  float magnet_uT_[3] = {0.0f, 0.0f, 0.0f};
  float chip_temp_c_ = 0.0f; /*!< 芯片温度 (°C) */
  uint32_t last_update_ms_ = 0u; /*!< 数据更新时间 */

  /* Flags */
  bool is_left_;
  bool is_enabled_ = false;
  bool is_data_fresh_ = false;
  bool is_quat_updated_ = false;
  float sagittal_direction_sign_ = 1.0f;
  SagittalSource sagittal_source_;

  bool IsUsable() const
  {
    return is_enabled_ && is_data_fresh_;
  }

  bool IsFresh(uint32_t now_ms) const
  {
    return last_update_ms_ > 0u && (now_ms - last_update_ms_) <= kDataTimeoutMs;
  }

  void MarkKinematicsUpdated(uint32_t now_ms)
  {
    last_update_ms_ = now_ms;
    is_data_fresh_ = true;
  }

  void UpdateFreshness(uint32_t now_ms)
  {
    is_data_fresh_ = IsFresh(now_ms);
    if (!is_data_fresh_)
    {
      is_sagittal_gyro_lpf_inited_ = false;
    }
  }

  void CaptureStandPosture()
  {
    if (!IsUsable() || is_stand_posture_valid_) return;

    roll_stand_rad_ = roll_rad_;
    pitch_stand_rad_ = pitch_rad_;
    yaw_stand_rad_ = yaw_rad_;
    sagittal_stand_ref_rad_ = sagittal_raw_rad_;
    sagittal_pos_rad_ = 0.0f;
    is_stand_posture_valid_ = true;
  }

  void ClearStandPosture()
  {
    roll_stand_rad_ = 0.0f;
    pitch_stand_rad_ = 0.0f;
    yaw_stand_rad_ = 0.0f;
    sagittal_stand_ref_rad_ = 0.0f;
    sagittal_pos_rad_ = 0.0f;
    is_stand_posture_valid_ = false;
  }

  float SagittalRawRad() const
  {
    return sagittal_raw_rad_;
  }

  float SagittalGyroRadps() const
  {
    return sagittal_gyro_lpf_radps_;
  }

  float SagittalGyroRawRadps() const
  {
    return sagittal_gyro_radps_;
  }

  float SagittalFromStandRefRad() const {return sagittal_pos_rad_;}
  float SagittalFromStandRefDeg() const {return SagittalFromStandRefRad() * RAD_TO_DEG;}

  bool UpdateEulerFromQuaternion()
  {
    const float norm_sq =
      q_[0] * q_[0] +
      q_[1] * q_[1] +
      q_[2] * q_[2] +
      q_[3] * q_[3];
    if (!isfinite(norm_sq) || norm_sq < 1.0e-6f)
    {
      return false;
    }

    const float inv_norm = 1.0f / sqrtf(norm_sq);
    q_[0] *= inv_norm;
    q_[1] *= inv_norm;
    q_[2] *= inv_norm;
    q_[3] *= inv_norm;
    Quaternion2EulerRad(q_, &roll_rad_, &pitch_rad_, &yaw_rad_);
    return true;
  }

  void ApplyPendingQuaternion(uint32_t now_ms)
  {
    if (!is_quat_updated_) return;
    is_quat_updated_ = false;
    if (!UpdateEulerFromQuaternion()) return;
    UpdateSagittalKinematics();
    MarkKinematicsUpdated(now_ms);
  }

  void UpdateSagittalKinematics()
  {
    switch (sagittal_source_)
    {
    case SagittalSource::kEulerPitch:
      sagittal_raw_rad_ = pitch_rad_;
      sagittal_gyro_radps_ = gyro_radps_[1];
      break;

    case SagittalSource::kEulerRoll:
      sagittal_raw_rad_ = roll_rad_;
      sagittal_gyro_radps_ = gyro_radps_[0];
      break;

    case SagittalSource::kGravityXz:
    {
      const float q0 = q_[0];
      const float q1 = q_[1];
      const float q2 = q_[2];
      const float q3 = q_[3];
      const float gx = 2.0f * (q1 * q3 - q0 * q2);
      const float gz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
      sagittal_raw_rad_ = atan2f(-gz, -gx);
      sagittal_gyro_radps_ = gyro_radps_[1];
      break;
    }

    case SagittalSource::kGravityYz:
    {
      const float q0 = q_[0];
      const float q1 = q_[1];
      const float q2 = q_[2];
      const float q3 = q_[3];
      const float gy = 2.0f * (q2 * q3 + q0 * q1);
      const float gz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
      sagittal_raw_rad_ = WrapPi(atan2f(gy, gz) + _PI / 2.0f);
      sagittal_gyro_radps_ = gyro_radps_[0];
      break;
    }

    default:
      break;
    }

    sagittal_raw_rad_ = WrapPi(sagittal_direction_sign_ * sagittal_raw_rad_);
    sagittal_gyro_radps_ *= sagittal_direction_sign_;
    sagittal_pos_rad_ = is_stand_posture_valid_ ?
                          WrapPi(sagittal_raw_rad_ - sagittal_stand_ref_rad_) :
                          0.0f;

    if (!is_sagittal_gyro_lpf_inited_)
    {
      sagittal_gyro_lpf_radps_ = sagittal_gyro_radps_;
      is_sagittal_gyro_lpf_inited_ = true;
    }
    else
    {
      sagittal_gyro_lpf_radps_ += sagittal_gyro_lpf_alpha_ *
                                   (sagittal_gyro_radps_ - sagittal_gyro_lpf_radps_);
    }
  }

private:
  static float ComputeLpfAlpha(float sample_rate_hz)
  {
    return sample_rate_hz > 0.0f ?
             1.0f - expf(-_2PI * 10.0f / sample_rate_hz) :
             1.0f;
  }

  float sagittal_gyro_lpf_alpha_ = 1.0f;
  bool is_sagittal_gyro_lpf_inited_ = false;
  static constexpr uint32_t kDataTimeoutMs = 200u;
};

/**
 * @brief 关节数据类
 * @note  1. 膝关节角度: 0° 表示膝伸直, 正值表示膝弯屈, 负值表示膝过伸
 *        2. 膝关节角速度: 正值表示膝弯屈, 负值表示膝伸展
 *        3. 膝关节力矩: 正值表示膝弯屈, 负值表示膝伸展
 */
class JointData
{
public:
  explicit JointData(bool is_left = true) :
    is_left_(is_left)
  {
  }
  virtual ~JointData() = default;

  float sagittal_pos_ctrlref_rad_ = 0.0f; /*!< 人体关节矢状面角度参考(控制输入) */
  float sagittal_pos_rad_ = 0.0f; /*!< 人体关节矢状面角度反馈 */
  float sagittal_vel_ctrlref_radps_ = 0.0f; /*!< 人体关节矢状面角速度参考(控制输入) */
  float sagittal_vel_radps_ = 0.0f; /*!< 人体关节矢状面角速度反馈 */
  float sagittal_vel_lpf_radps_ = 0.0f; /*!< 人体关节矢状面角速度10 Hz低通反馈 */
  float sagittal_pos_offset_rad_ = 0.0f; /*!< 人体关节矢状面角度偏移/中立位偏置 */
  bool is_sagittal_pos_offset_valid_ = false; /*!< true: sagittal_pos_offset_rad_ 已捕获 */

  float motor_to_joint_sign_ = 1.0f; /*!< 电机反馈到膝关节坐标的方向符号: +1 电机正向=膝屈曲, -1 电机正向=膝伸展 */
  float link_pos_ctrlref_rad_ = 0.0f; /*!< 连杆关节角度参考 */
  float link_pos_rad_ = 0.0f; /*!< 连杆关节角度反馈 */
  float link_vel_ctrlref_radps_ = 0.0f; /*!< 连杆关节角速度参考 */
  float link_vel_radps_ = 0.0f; /*!< 连杆关节角速度反馈 */
  float link_vel_lpf_radps_ = 0.0f; /*!< 连杆关节角速度10 Hz低通反馈 */
  float link_pos_offset_rad_ = 0.0f; /*!< 连杆关节角度偏移, 用于标定 */
  bool is_link_pos_offset_valid_ = false; /*!< true: link_pos_offset_rad_ 已捕获 */

  float tor_bio_Nm_ = 0.0f; /*!< 生物力矩, 由人体运动产生 */
  float tor_interact_ref_Nm_ = 0.0f; /*!< 人机交互力矩参考 */
  float tor_interact_Nm_ = 0.0f; /*!< 人机交互力矩反馈 */
  float tor_output_ref_Nm_ = 0.0f; /*!< 关节输出力矩参考 */
  float tor_output_Nm_ = 0.0f; /*!< 关节输出力矩反馈 */

  /* 标志位 */
  bool is_left_;
  bool is_actuator_enabled_ = false;

  void UpdateSagittalVelocity(float velocity_radps, uint32_t sample_ms)
  {
    sagittal_vel_radps_ = velocity_radps;
    if (!sagittal_vel_lpf_inited_)
    {
      sagittal_vel_lpf_radps_ = velocity_radps;
      sagittal_vel_lpf_sample_ms_ = sample_ms;
      sagittal_vel_lpf_inited_ = true;
    }
    else if (sample_ms != sagittal_vel_lpf_sample_ms_)
    {
      sagittal_vel_lpf_radps_ += kSagittalVelocityLpfAlpha * (velocity_radps - sagittal_vel_lpf_radps_);
      sagittal_vel_lpf_sample_ms_ = sample_ms;
    }
  }

  void UpdateLinkVelocity(float velocity_radps)
  {
    link_vel_radps_ = velocity_radps;
    if (!link_vel_lpf_inited_)
    {
      link_vel_lpf_radps_ = velocity_radps;
      link_vel_lpf_inited_ = true;
    }
    else
    {
      link_vel_lpf_radps_ += kLinkVelocityLpfAlpha * (velocity_radps - link_vel_lpf_radps_);
    }
  }

  void ClearStandPosture()
  {
    sagittal_pos_rad_ = 0.0f;
    sagittal_vel_radps_ = 0.0f;
    sagittal_vel_lpf_radps_ = 0.0f;
    sagittal_pos_offset_rad_ = 0.0f;
    is_sagittal_pos_offset_valid_ = false;
    sagittal_vel_lpf_sample_ms_ = 0u;
    sagittal_vel_lpf_inited_ = false;

    link_pos_rad_ = 0.0f;
    link_vel_radps_ = 0.0f;
    link_vel_lpf_radps_ = 0.0f;
    link_pos_offset_rad_ = 0.0f;
    is_link_pos_offset_valid_ = false;
    link_vel_lpf_inited_ = false;
  }

private:
  static constexpr float kSagittalVelocityLpfAlpha = 0.2695973f; /* cutoff 10 Hz, sample rate 200 Hz */
  static constexpr float kLinkVelocityLpfAlpha = 0.0608986f; /* cutoff 10 Hz, sample rate 1 kHz */
  uint32_t sagittal_vel_lpf_sample_ms_ = 0u;
  bool sagittal_vel_lpf_inited_ = false;
  bool link_vel_lpf_inited_ = false;
};

class AnkleData : public JointData
{
public:
  explicit AnkleData(bool is_left = true) :
    JointData(is_left)
  {
  }
  virtual ~AnkleData() = default;

  float plantarflexion_force_N_ = 0.0f; /*!< 跖屈拉力 (N), 由拉力传感器经线性变换得到 */
  float plantarflexion_moment_arm_m_ = 0.1f; /*!< 跖屈力矩臂 (m), 由解算得到 */
  float dorsiflexion_force_N_ = 0.0f; /*!< 背屈拉力 (N), 由拉力传感器经线性变换得到 */
  float dorsiflexion_moment_arm_m_ = 0.1f; /*!< 背屈力矩臂 (m), 由解算得到 */
};

/**
 * @brief SEA (串联弹性驱动器) 膝关节数据
 * @note  膝关节通过丝杠-弹簧串联结构驱动, 因此需要额外的滑块位置、弹簧力等参数
 *        坐标系: pos_linear_encoder_mm_ 在最大伸展时为 0, 弯曲时增大
 */
class KneeSeaJointData : public JointData
{
public:
  explicit KneeSeaJointData(bool is_left = true) :
    JointData(is_left)
  {
  }
  virtual ~KneeSeaJointData() = default;

  float pos_slider_mm_ = 0.0f; /*!< 滑块位移 (mm), 由电机角度换算 */
  float pos_slider_offset_mm_ = 0.0f; /*!< 滑块在最大膝伸展 (0度) 时的偏置位置 */
  float vel_slider_mmps_ = 0.0f; /*!< 滑块线速度 (mm/s) */
  float screw_lead_rad2mm_ = 2.0f / _2PI; /*!< 丝杠导程系数: rad → mm */

  float pos_linear_encoder_mm_ = 0.0f; /*!< 外框位移 (mm), 由磁栅尺编码器测量 */
  float pos_linear_encoder_offset_mm_ = 13587.649414f; /*!< 外框在最大膝伸展时的编码器读数 */
  float max_pos_linear_encoder_mm_ = 13636.950195f; /*!< 外框在最大膝弯曲 (~90度) 时的编码器读数 */
  float vel_linear_encoder_mmps_ = 0.0f; /*!< 外框线速度 (mm/s) */

  float pos_bias_mm_ = 0.0f; /*!< 滑块与外框位移之差 (弹簧压缩量) */
  float force_spring_ref_N_ = 0.0f; /*!< 弹簧参考力 (N) */
  float force_spring_N_ = 0.0f; /*!< 弹簧反馈力 = pos_bias_mm_ × stiffness */
  float spring_stiffness_Npmm_ = 2 * 15.637f; /*!< 两根弹簧的总刚度 (N/mm) */
};

/* ============================================================================
 * 4. Gait / Stair / STS / AO Data
 * ========================================================================== */

struct FsrSensorData
{
  float raw_reading = 0.0f; /*!< 原始读数 */
  float force_N = 0.0f;
  bool ground_contact = false; /*!< 当前着地状态 (施密特触发器输出) */
  float contact_norm = 0.0f; /*!< 归一化接触读数 [0, 1.5], = (raw - baseline) / range */

  /* 状态约束自适应接触检测 */
  float contact_baseline = 0.0f;
  float contact_peak_ref = 1.0f;
  float contact_peak_step = 0.0f;
  bool contact_seed_enabled = false; /*!< 使用经验 baseline/peak_ref 初始化接触检测 */
  float contact_seed_baseline = 50.0f;
  float contact_seed_peak_ref = 600.0f;
  float contact_on_ratio = 0.30f;
  float contact_off_ratio = 0.12f;
  float contact_baseline_alpha = 0.005f;
  float contact_peak_alpha = 0.20f;
  uint8_t contact_on_count = 0;
  uint8_t contact_off_count = 0;
  uint8_t contact_on_count_threshold = 2;
  uint8_t contact_off_count_threshold = 4;
  bool contact_inited = false;
  float contact_min_range = 150.0f;
};

/**
 * @brief FSR 主步态相位枚举
 * @note  仅保留 Unknown/Stance/Swing, 不再细分旧子相.
 */
enum class GaitPhase : uint8_t
{
  kUnknown = 0,
  kStance,
  kSwing,
};

enum GaitEvent : uint8_t
{
  kFS = 0, /*!< Foot Strike: foot_contact rising */
  kHS, /*!< Heel Strike: heel_contact rising */
  kTS, /*!< Toe Strike: toe_contact rising */
  kFO, /*!< Foot Off: foot_contact falling */
  kHO, /*!< Heel Off: heel_contact falling */
  kTO, /*!< Toe Off: toe_contact falling */
  kNumGaitEvents,
};

/**
 * @brief 单侧 FSR 步态估计数据 —— FS/FO + Stance/Swing 系统
 * @note  FS/FO 是主事件, 用于支撑/摆动相切换和百分比计算.
 *        HS/TS/HO/TO 是局部接触事件, 主要用于调试、落脚类型判断和模式识别.
 *
 *        percent_gait_ 基于 FS -> next FS 计算, 初始值 -1 表示无效
 */
class FsrGaitData
{
public:
  explicit FsrGaitData(bool is_left = true) :
    is_left_(is_left)
  {
  }
  virtual ~FsrGaitData() = default;

  bool IsFresh(uint32_t now_ms) const
  {
    return last_update_ms_ > 0u &&
           now_ms >= last_update_ms_ &&
           (now_ms - last_update_ms_) <= kFsrDataTimeoutMs;
  }

  bool IsUsable() const
  {
    return IsContactReady();
  }

  bool IsContactReady() const
  {
    return is_enabled_ && is_data_fresh_ && heel_.contact_inited && toe_.contact_inited;
  }

  static constexpr uint8_t kNumStepsAvg = 3; /*!< 步态周期滑动平均窗口大小 */
  static constexpr uint32_t kFsrDataTimeoutMs = 500u; /*!< FSR 数据超时阈值 (ms), 超过此时间未更新则判定失联 */

  FsrSensorData heel_; /*!< 足跟 FSR 传感器数据 */
  FsrSensorData toe_; /*!< 足尖 FSR 传感器数据 */
  float cop_x_mm_ = 0.0f;
  float cop_y_mm_ = 0.0f;
  float vgrf_N_ = 0.0f;
  uint32_t last_update_ms_ = 0;

  /* 步态事件时间戳与相位时长窗口 */
  uint32_t event_ts_ms_[kNumGaitEvents] = {0}; /*!< 最近一次各事件的发生时间戳 (ms) */
  uint32_t prev_event_ts_ms_[kNumGaitEvents] = {0}; /*!< 前一次各事件的发生时间戳 (ms) */
  uint32_t event_count_[kNumGaitEvents] = {0}; /*!< 各事件累计触发次数, 用于低频遥测诊断 */
  uint32_t gait_duration_window_ms_[kNumStepsAvg] = {0}; /*!< FS->next FS 滑动窗口 (ms) */
  uint32_t stance_duration_window_ms_[kNumStepsAvg] = {0}; /*!< FS->FO 滑动窗口 (ms) */
  uint32_t swing_duration_window_ms_[kNumStepsAvg] = {0}; /*!< FO->next FS 滑动窗口 (ms) */
  float expected_gait_duration_ms_ = -1.0f; /*!< 预期步态周期时长 (FS->next FS) (ms) */
  float expected_stance_duration_ms_ = -1.0f; /*!< 预期支撑相时长 (FS->FO) (ms) */
  float expected_swing_duration_ms_ = -1.0f; /*!< 预期摆动相时长 (FO->next FS) (ms) */
  GaitPhase current_phase_ = GaitPhase::kUnknown;
  GaitPhase prev_phase_ = GaitPhase::kUnknown;
  bool phase_changed_ = false; /*!< 本周期相位是否发生变化 (上升沿) */
  bool is_data_fresh_ = false; /*!< 本周期 FSR 数据是否新鲜有效 */
  bool is_phase_valid_ = false; /*!< 本周期 FSR 步态相位/百分比是否可用于控制 */

  float percent_gait_ = -1.0f; /*!< 步态周期百分比 [0, 100], -1 表示无效 */
  float percent_stance_ = -1.0f; /*!< 支撑相百分比 [0, 100] */
  float percent_swing_ = -1.0f; /*!< 摆动相百分比 [0, 100] */

  /* 滑动平均过滤参数 */
  float expected_duration_window_upper_coeff_ = 1.75f; /*!< 异常值过滤: 上界系数 (× max) */
  float expected_duration_window_lower_coeff_ = 0.25f; /*!< 异常值过滤: 下界系数 (× min) */

  /* FSR 着地状态与边沿检测 */
  bool heel_contact_state_ = false; /*!< 足跟当前着地状态 */
  bool toe_contact_state_ = false; /*!< 足尖当前着地状态 */
  bool foot_contact_state_ = false;
  bool prev_heel_contact_state_ = false; /*!< 足跟上周期着地状态 */
  bool prev_toe_contact_state_ = false; /*!< 足尖上周期着地状态 */
  bool prev_foot_contact_state_ = false;

  /* 本周期检测到的步态事件标志 */
  bool event_fs_ = false; /*!< Foot Strike */
  bool event_hs_ = false; /*!< Heel Strike */
  bool event_ts_ = false; /*!< Toe Strike */
  bool event_fo_ = false; /*!< Foot Off */
  bool event_ho_ = false; /*!< Heel Off */
  bool event_to_ = false; /*!< Toe Off */

  bool is_left_; /*!< 是否为左侧 */
  bool is_enabled_ = false; /*!< 是否启用步态估计 */
};

enum class StairPhase : uint8_t
{
  kSwing = 0, /*!< 摆动相 (空中) */
  kWeightAcceptance, /*!< 承重相 (刚触地，缓冲) */
  kPullUp, /*!< 拉升相 (仅上楼梯：对抗重力伸膝爆发做功) */
  kControlledLowering, /*!< 受控下降相 (仅下楼梯：离心收缩高阻尼) */
  kForwardContinuance /*!< 前向过渡相 (重心越过支撑腿) */
};

class StairPhaseData
{
public:
  explicit StairPhaseData(bool is_left = true) :
    is_left_(is_left)
  {
  }
  virtual ~StairPhaseData() = default;

  StairPhase current_phase_ = StairPhase::kSwing; /*!< 当前楼梯步态相位 */
  StairPhase prev_phase_ = StairPhase::kSwing; /*!< 上一周期楼梯步态相位 */
  bool phase_changed_ = false; /*!< 本周期相位是否发生变化 (上升沿) */
  bool is_valid_ = false; /*!< 本周期楼梯相位估计是否有效 */
  bool is_contact_ = false; /*!< 本周期足底是否接触 */
  bool prev_is_contact_ = false; /*!< 上一周期足底是否接触 */

  float contact_start_knee_angle_deg_ = 0.0f; /*!< 本次接触上升沿的膝角 (deg) */
  float min_knee_angle_since_contact_deg_ = 0.0f; /*!< 本次接触以来的最小膝角 (deg) */

  bool is_left_; /*!< 是否为左侧 */
  bool is_enabled_ = false; /*!< 是否启用楼梯步态估计 */
};

enum class StsPhase : uint8_t
{
  kSeatOff = 0, /*!< 臀部离座 (刚切入起立模式时的初始状态) */
  kExtension, /*!< 向上伸展 (发力站起) */
  kStabilization, /*!< 直立稳定 (准备切入 kStanding) */
  kYielding, /*!< 屈膝退让 (坐下时的阻尼支撑) */
  kSeated /*!< 完全落座 (准备切入 kSitting) */
};

class StsPhaseData
{
public:
  explicit StsPhaseData()
  {
  }
  virtual ~StsPhaseData() = default;

  StsPhase current_phase_ = StsPhase::kSeatOff; /*!< 当前 STS 相位 */
  StsPhase prev_phase_ = StsPhase::kSeatOff; /*!< 上一周期 STS 相位 */
  bool phase_changed_ = false; /*!< 本周期相位是否发生变化 (上升沿) */
  bool is_valid_ = false; /*!< 本周期 STS 相位估计是否有效 */
  bool is_feet_loaded_ = false; /*!< 双脚是否检测到承重/接触 */

  uint8_t valid_side_count_ = 0; /*!< 参与 STS 运动学估计的有效侧数量 */
  float avg_thigh_pitch_deg_ = 0.0f; /*!< 双侧/可用侧平均大腿俯仰角 (deg) */
  float avg_thigh_vel_degps_ = 0.0f; /*!< 双侧/可用侧平均大腿角速度 (deg/s) */
  float avg_knee_angle_deg_ = 0.0f; /*!< 双侧/可用侧平均膝角估计 (deg) */
  float avg_knee_vel_degps_ = 0.0f; /*!< 双侧/可用侧平均膝角速度估计 (deg/s) */
  float percent_transition_ = -1.0f; /*!< 坐站/站坐完整转换进度 [0, 100], -1 表示无效 */

  bool is_enabled_ = false; /*!< 是否启用STS估计 */
};

/**
 * @brief 自适应振荡器 (AO) 输出数据
 * @note  left/right_phi_comp_rad_ 是补偿后的步态相位 [0, 2π),
 *        可用于替代 FSR 步态百分比进行相位同步助力
 */
class AoData
{
public:
  explicit AoData() = default;
  virtual ~AoData() = default;

  uint32_t left_event_cnt_ = 0; /*!< 左侧步态事件累计计数 */
  uint32_t right_event_cnt_ = 0; /*!< 右侧步态事件累计计数 */
  float left_phi_comp_rad_ = 0.0f; /*!< 左侧补偿后步态相位 (rad) */
  float right_phi_comp_rad_ = 0.0f; /*!< 右侧补偿后步态相位 (rad) */
  float teach_signal_rad_ = 0.0f; /*!< 当前 AO 示教信号: left thigh roll - right thigh roll (rad) */
  float fitted_signal_rad_ = 0.0f; /*!< 当前 AO 傅里叶重构信号 (rad) */
  float frequency_hz_ = 0.0f; /*!< 当前 AO 自适应频率 (Hz) */
  bool is_valid_ = false; /*!< AO 是否已有双侧事件锚点, 相位可用于控制 */

  bool is_enabled_ = false; /*!< 是否启用自适应振荡器 */
};

/* ============================================================================
 * 5. SideData And ExoData
 * ========================================================================== */

/**
 * @brief 单侧 (左/右) 全部感知与控制数据的聚合容器
 * @note  包含髋/膝/SEA膝/踝关节的参考值与反馈值,
 *        FSR 步态数据, 以及足部/小腿/大腿三颗 IMU 的姿态数据
 */
class SideData
{
public:
  explicit SideData(bool is_left = true) :
    hip_joint_(is_left),
    knee_joint_(is_left),
    knee_sea_joint_(is_left),
    ankle_joint_(is_left),
    fsr_gait_data_(is_left),
    stair_phase_data_(is_left),
    foot_imu_(is_left, is_left ? 1.0f : -1.0f, ImuData::SagittalSource::kEulerPitch),
    shank_imu_(is_left, 1.0f, ImuData::SagittalSource::kEulerRoll),
    thigh_imu_(is_left, 1.0f, ImuData::SagittalSource::kEulerRoll),
    is_left_(is_left)
  {
  }
  virtual ~SideData() = default;

  JointData hip_joint_; /*!< 髋关节数据 (DM4340 电机) */
  JointData knee_joint_; /*!< 膝关节数据 (Robstride RS01 电机) */
  KneeSeaJointData knee_sea_joint_; /*!< SEA 膝关节数据 (大疆电调 + 磁栅尺) */
  AnkleData ankle_joint_; /*!< 踝关节数据 (Robstride RS02 电机) */
  FsrGaitData fsr_gait_data_; /*!< FSR 步态估计数据 */
  StairPhaseData stair_phase_data_; /*!< 楼梯步态相位数据 */

  ImuData foot_imu_; /*!< 足部 IMU (NRF54 板载) */
  ImuData shank_imu_; /*!< 小腿 IMU (达妙 CAN IMU) */
  ImuData thigh_imu_; /*!< 大腿 IMU (达妙 CAN IMU) */

  bool is_left_; /*!< 是否为左侧 */
  bool is_calibrated_ = false; /*!< 该侧全部传感器/关节是否均已标定 */
};

/**
 * @brief 运动模式枚举 —— 决定中层控制策略
 */
enum class LocoMode : uint8_t
{
  kSitting = 0, /*!< 坐姿 */
  kSitToStand, /*!< 坐→站转换 */
  kStanding, /*!< 站立 */
  kStandToSit, /*!< 站→坐转换 */
  kWalking, /*!< 平地行走 (已实现) */
  kRampAscent, /*!< 上坡 */
  kRampDescent, /*!< 下坡 */
  kStairAscent, /*!< 上楼梯 */
  kStairDescent, /*!< 下楼梯 */
  kDebug,
};

/**
 * @brief 意图识别/运动模式识别层共享数据
 */
class IntentionData
{
public:
  struct SideFootRef
  {
    float terrain_slope_deg = 0.0f;
    bool terrain_slope_valid = false;
  };

  /* Mode arbitration — 算法产出 */
  LocoMode detected_mode_ = LocoMode::kWalking;
  LocoMode current_mode_ = LocoMode::kWalking;
  LocoMode prev_mode_ = LocoMode::kWalking;

  /* Foot & terrain (per-side) */
  SideFootRef foot_ref[2] = {};

  /* Terrain slope (bilateral aggregate) */
  float terrain_slope_deg_ = 0.0f;
  float terrain_slope_confidence_ = 0.0f;
  bool terrain_slope_valid_ = false;

  /* Dataset labels */
  LocoMode label_mode_ = LocoMode::kWalking;
  float label_slope_deg_ = 0.0f;
  bool label_is_valid_ = false;

  /* ——— 物理规则状态机运行时状态 ——— */
  uint32_t double_support_timer_ms_ = 0; /*!< 双支撑持续时间 (ms) */
  bool sit_to_stand_intent_detected_ = false; /*!< 检测到坐→站意图 */
  bool stand_to_sit_intent_detected_ = false; /*!< 检测到站→坐意图 */
  uint32_t sit_still_start_ms_ = 0; /*!< 坐下完成: 膝屈曲到位+静止计时起点 */

  bool is_enabled_ = false; /*!< 是否启用意图识别/模式识别功能 */

  void Reset()
  {
    detected_mode_ = LocoMode::kWalking;
    current_mode_ = LocoMode::kWalking;
    prev_mode_ = LocoMode::kWalking;
    foot_ref[0] = {};
    foot_ref[1] = {};
    terrain_slope_deg_ = 0.0f;
    terrain_slope_confidence_ = 0.0f;
    terrain_slope_valid_ = false;
    label_mode_ = LocoMode::kWalking;
    label_slope_deg_ = 0.0f;
    label_is_valid_ = false;
    double_support_timer_ms_ = 0;
    sit_to_stand_intent_detected_ = false;
    stand_to_sit_intent_detected_ = false;
    sit_still_start_ms_ = 0;
  }
};

/**
 * @brief 外骨骼系统全局数据中心
 * @note
 */
class ExoData
{
public:
  /**
   * @brief 系统顶层状态机状态枚举
   * @note  除kEstopped外状态值直接映射到 WS2812 LED 颜色索引
   */
  enum class State : uint8_t
  {
    kSleep = 0U, /*!< 休眠: 电机关断, 等待 wakeup 命令 */
    kWaitMotorComm, /*!< 等待电机通信建立, 收到 calib 命令后进入标定 */
    kCalibrating, /*!< 标定中: 关节零位 + FSR 踩踏力范围标定 */
    kReady, /*!< 就绪: 标定完成, 零力控制, 等待 start 命令 */
    kAssisting, /*!< 助力中: 执行步态估计 + 关节助力控制 */
    kFaultLowBattery, /*!< 故障-欠压: 电池电压 < 19.0V, 电机关断 */
    kFaultSystem, /*!< 故障-系统: 电机/通信/IMU 等硬件故障, 电机关断 */
    kEstopped, /*!< 急停: 最高优先级, 立即切断所有电机输出, 仅允许断电重启恢复 */
  };

  /**
   * @brief 系统错误码 (位掩码, 可同时指示多个故障)
   */
  enum class Error : uint32_t
  {
    kNone = 0 << 0, /*!< 无错误 */
    kBatteryLow = 1 << 0, /*!< 电池电压 < 19.0V */
    kLeftHipFault = 1 << 1, /*!< 左髋关节电机故障 (暂未检测) */
    kRightHipFault = 1 << 2, /*!< 右髋关节电机故障 (暂未检测) */
    kLeftKneeFault = 1 << 3, /*!< 左膝关节电机故障 (error_code / fault_code 非零) */
    kRightKneeFault = 1 << 4, /*!< 右膝关节电机故障 */
    kLeftAnkleFault = 1 << 5, /*!< 左踝关节电机故障 */
    kRightAnkleFault = 1 << 6, /*!< 右踝关节电机故障 */
  };

  /**
   * @brief 系统事件 (位掩码) —— 由 Shell 命令或内部逻辑触发
   * @note  每周期 Run() 开头通过 AllowedEventsForState() 过滤非法事件,
   *        确保状态机不会收到当前状态下不应该处理的事件
   */
  enum class SysEvent : uint32_t
  {
    kNone = 0 << 0, /*!< 无事件 */
    kEmergencyStop = 1 << 0, /*!< 急停 (所有状态均允许, 最高优先级) */
    kWakeup = 1 << 1, /*!< 从 kSleep 唤醒 */
    kStartCalibrate = 1 << 2, /*!< 开始标定 (kWaitMotorComm / kReady) */
    kStartAssist = 1 << 3, /*!< 开始助力 (kReady) */
    kStopAssist = 1 << 4, /*!< 停止助力 (kAssisting) */
    kEnterSleep = 1 << 5, /*!< 进入休眠 (大部分非故障状态允许) */
    kClearFaults = 1 << 6, /*!< 清除故障 (kFaultSystem) */
  };

  /**
   * @brief VOFA 遥测配置
   */
  struct TelemetryConfig
  {
    bool enable = true; /*!< 是否启用遥测发送 */
    uint32_t pause_until_ms = 0; /*!< 暂停遥测直到此时刻 (Shell 交互后留出发送窗口) */
  };

  ExoData() :
    left_side_(true),
    right_side_(false)
  {
  }
  ~ExoData() = default;

  bool HasFullStandPostureRef() const
  {
    const bool body_ok = body_imu_.IsUsable() && body_imu_.is_stand_posture_valid_;

    const bool left_ok =
      left_side_.thigh_imu_.IsUsable() &&
      left_side_.thigh_imu_.is_stand_posture_valid_ &&
      left_side_.shank_imu_.IsUsable() &&
      left_side_.shank_imu_.is_stand_posture_valid_ &&
      left_side_.foot_imu_.IsUsable() &&
      left_side_.foot_imu_.is_stand_posture_valid_ &&
      left_side_.hip_joint_.is_sagittal_pos_offset_valid_ &&
      left_side_.knee_joint_.is_sagittal_pos_offset_valid_ &&
      left_side_.ankle_joint_.is_sagittal_pos_offset_valid_;

    const bool right_ok =
      right_side_.thigh_imu_.IsUsable() &&
      right_side_.thigh_imu_.is_stand_posture_valid_ &&
      right_side_.shank_imu_.IsUsable() &&
      right_side_.shank_imu_.is_stand_posture_valid_ &&
      right_side_.foot_imu_.IsUsable() &&
      right_side_.foot_imu_.is_stand_posture_valid_ &&
      right_side_.hip_joint_.is_sagittal_pos_offset_valid_ &&
      right_side_.knee_joint_.is_sagittal_pos_offset_valid_ &&
      right_side_.ankle_joint_.is_sagittal_pos_offset_valid_;

    return body_ok && left_ok && right_ok;
  }

  SideData left_side_; /*!< 左侧数据 (is_left = true) */
  SideData right_side_; /*!< 右侧数据 (is_left = false) */
  AoData ao_data_; /*!< 自适应振荡器输出数据 */
  StsPhaseData sts_phase_data_; /*!< 坐立转换相位数据 */
  ImuData body_imu_{true, 1.0f, ImuData::SagittalSource::kGravityXz, 1000.0f}; /*!< 躯干 IMU 数据 (BMI088 + Mahony 滤波器) */
  IntentionData intention_data_; /*!< 意图识别/运动模式识别层数据 */

  State state_ = State::kSleep; /*!< 当前系统状态 */
  Error error_code_ = Error::kNone; /*!< 当前错误码 (位掩码) */
  SysEvent pending_events_ = SysEvent::kNone; /*!< 待处理事件 (位掩码) */

  TelemetryConfig telemetry_config_{.enable = true, .pause_until_ms = 0}; /*!< 遥测配置 */

  struct UserInfo
  {
    float weight_kg_ = 60.0f;
    float height_m_ = 1.65f;
    float thigh_length_m = 0.47f;
    float shank_length_m = 0.40f;
  } user_info_;

  float battery_voltage_ = 24.0f; /*!< 电池电压 (V), 由 ADC 读取 */
  bool do_test = false; /*!< 测试模式开关 (testsw 命令切换) */
};

DEFINE_ENUM_CLASS_BITWISE_OPS(ExoData::Error)
DEFINE_ENUM_CLASS_BITWISE_OPS(ExoData::SysEvent)

/* ============================================================================
 * 6. Hardware Mapping
 * ========================================================================== */

/**
 * @brief 外骨骼硬件外设句柄聚合结构体
 * @note  将 CubeMX 生成的外设句柄集中传递给 Exo 构造函数,
 *        避免 Exo 类直接依赖 CubeMX 全局变量
 */
struct ExoHardware
{
  FDCAN_HandleTypeDef &motor_can1; /*!< 左侧电机总线 */
  FDCAN_HandleTypeDef &motor_can2; /*!< 右侧电机总线 */
  FDCAN_HandleTypeDef &sensor_can; /*!< FDCAN3: CAN传感器 */
  SPI_HandleTypeDef &sensor_spi; /*!< SPI3: NRF54 足部传感器 (备用通道) */
  UART_HandleTypeDef &sensor_uart; /*!< UART8: NRF54 足部传感器 (主通道) */
  UART_HandleTypeDef &shell_uart; /*!< UART9: 调试 Shell / 蓝牙透传 */
  UART_HandleTypeDef &left_mag_encoder_uart; /*!< USART2: 左膝磁栅尺编码器 */
  UART_HandleTypeDef &right_mag_encoder_uart; /*!< USART3: 右膝磁栅尺编码器 */
};

/**
 * @brief 各关节电机的 CAN 总线 ID 定义
 */
enum ExoJointCanID : uint8_t
{
  kLeftHip = 0x01, /*!< 左髋: 达妙 DM4340 */
  kRightHip = 0x02, /*!< 右髋: 达妙 DM4340 */
  kLeftKnee = 0x2B, /*!< 左膝: 灵足 Robstride RS01 */
  kRightKnee = 0x56, /*!< 右膝: 灵足 Robstride RS01 */
  kLeftAnkle = 0x2A, /*!< 左踝: 灵足 Robstride RS02 */
  kRightAnkle = 0x55, /*!< 右踝: 灵足 Robstride RS02 */
};

/* ============================================================================
 * 7. Joint Modules
 * ========================================================================== */

/**
 * @brief 踝关节控制器 (Robstride RS02 电机, 绳索牵引方式)
 * @note  通过步态相位控制绳索位置:
 *        - 摆动相: cable_released_position_ (绳索松弛)
 *        - 支撑相初期: cable_pre_tensioned_position_ (预张紧)
 *        - 支撑相后期 (助力区间): cable_tensioned_position_ (绳索拉紧, 助力跖屈)
 */
class AnkleJoint
{
public:
  explicit AnkleJoint(bool is_left, ExoData &pe, FDCAN_HandleTypeDef &motor_can) :
    pe_(pe),
    ps_(is_left ? pe_.left_side_ : pe_.right_side_),
    pj_(is_left ? pe_.left_side_.ankle_joint_ : pe_.right_side_.ankle_joint_),
    motor_(motor_can, is_left ? ExoJointCanID::kLeftAnkle : ExoJointCanID::kRightAnkle)
  {
  }
  virtual ~AnkleJoint() = default;
  void Calibrate();
  void Read();
  bool IsMotorConnect();
  void Shutdown();
  void Standby();
  void Assist();

  ExoData &pe_;
  SideData &ps_;
  JointData &pj_;
  Robstride motor_;
  AnkleForceProfileGenerator force_profile_generator_;

  /* 踝关节控制参数 */
  float cable_released_position_ = 0.2f;
  float cable_pre_tensioned_position_ = 0.4f;
  float cable_tensioned_position_ = 1.6f;
  float assistance_start_phase_percent_ = 35.0f;
  float assistance_end_phase_percent_ = 65.0f;
};

class KneeJoint
{
public:
  enum class CtrlMode : uint8_t
  {
    kImpedance,
    kPosition,
    kOpenLoopTorque,
    kClosedLoopTorque,
  };

  explicit KneeJoint(bool is_left, ExoData &pe, FDCAN_HandleTypeDef &motor_can) :
    pe_(pe),
    ps_(is_left ? pe.left_side_ : pe.right_side_),
    pj_(is_left ? pe.left_side_.knee_joint_ : pe.right_side_.knee_joint_),
    motor_(motor_can, is_left ? ExoJointCanID::kLeftKnee : ExoJointCanID::kRightKnee),
    joint_tor_pid_(2.5f, 0.0f, 0.0f, -1.0f, motor_.limit_current_)
  {
  }
  virtual ~KneeJoint() = default;
  void Calibrate();
  void Read();
  void PrepareMotorConnectionCheck();
  bool IsMotorConnect();
  void Shutdown();
  void Standby();
  void Assist();
  void ApplyDivekarControl();

  ExoData &pe_;
  SideData &ps_;
  JointData &pj_;
  Robstride motor_;
  PIDController joint_tor_pid_;
  KneeForceProfileGenerator force_profile_generator_;

  void ClosedLoopTorqueControl();
  void OpenLoopTorqueControl();

  struct DivekarParams
  {
    /* false: thigh IMU + shank IMU; true: thigh IMU + knee link feedback */
    bool use_thigh_imu_and_link_feedback = true;
    /* false: final torque feedforward; true: motor-side impedance parameters */
    bool use_motion_control_impedance_output = false;
    float assistance_scale = 0.3f; /*!< Final Divekar output scale, range [0, 1]. */

    /* Stance: ascent spring */
    float k_a = 50.0f; /*!<ascent spring stiffness, Nm/rad */
    float m_theta_la = -50.0f; /*!< leg angle sigmoid slope */
    float d_theta_la = 0.2f; /*!< leg angle sigmoid offset, rad */

    /* Stance: non-ascent spring-damper */
    float k_na = 40.0f; /*!< non-ascent spring stiffness, Nm/rad */
    float m_na = -15.0f; /*!< non-ascent stiffness taper sigmoid slope */
    float d_na = 0.47f; /*!< non-ascent stiffness taper sigmoid offset, rad */
    float c_na = 1.0f; /*!< non-ascent damping coefficient, Nm*s/rad */

    /* Stance: lifting/lowering spring */
    float k_LL = 76.0f; /*!< Nm/rad */
    float m_LL1 = -1.0f;
    float m_LL2 = 5.0f;
    float d_LL = 1.0f; /*!< rad */
    float x1 = 2.0f;
    float x2 = -1.5f;

    /* Stance: gravity compensation */
    float g_st = 34.0f; /*!< Nm */

    /* Task sensitization */
    float m_heel = 75.0f; /*!< heel-loaded sigmoid slope for TLL and Tna */
    float d_heel = 0.35f; /*!< heel-loaded sigmoid offset, BW */
    float m_grf_unloaded = 75.0f; /*!< total-GRF unloaded sigmoid slope magnitude */
    float d_grf_unloaded = 0.35; /*!< total-GRF unloaded sigmoid offset, BW */
    float m_ajc_dist = 5.0f; /*!< AJCdist sigmoid slope */
    float d_ajc_dist = 14.0f; /*!< AJCdist sigmoid offset, cm */
    float m_ajc_y_a = 5.0f; /*!< AJCy sigmoid slope for ascent spring */
    float d_ajc_y_a = 4.0f; /*!< AJCy sigmoid offset for ascent spring, cm */
    float m_ajc_y_na = -10.0f; /*!< AJCy sigmoid slope for non-ascent spring */
    float d_ajc_y_na = 3.0f; /*!< AJCy sigmoid offset for non-ascent spring, cm */

    /* ZZZ: STS */
    float k_sts = 20.0f; /*!< Nm/rad */
    float k_stand2sit = 5.0f; /*!< Nm/rad */
    float c_sts = 6.0f; /*!< Nm*s/rad */
    float m_sts_foot_loaded = 50.0f;
    float d_sts_foot_loaded = 0.48f; /*!< BW */
    float m_sts_knee_raise_vel = -60.0f;
    float d_sts_knee_raise_vel = -0.225f; /*!< rad/s, knee extension is negative */
    float m_sts_knee_lower_vel = 60.0f;
    float d_sts_knee_lower_vel = 0.175f; /*!< rad/s, knee flexion is positive */
    float m_sts_knee_pos_sym = -50.0f;
    float d_sts_knee_pos_sym = 0.3f; /*!< rad */
    float m_sts_thigh_flex = 60.0f;
    float d_sts_thigh_flex = 0.25f; /*!< rad */
    float m_sts_deep_flex_decay = -10.0f;
    float d_sts_deep_flex_decay = 1.50f; /*!< rad */

    /* Swing: gravity / inertia / spring-damper */
    float g_sw = 8.0f; /*!< Nm */
    float a_sw = 2.0f; /*!< Nm */
    float x3 = 0.2f;
    float k_sw = 0.6f; /*!< Nm/rad */
    float x4 = 3.0f;
    float theta_k_eq = 0.17f; /*!< rad */
    float c_sw = 0.2f; /*!< Nm*s/rad */

    /**
     * @note The supplementary table gives g_sw but does not list m_grav_sw and d_grav_sw.
     *       Keep them configurable and tune them on your hardware.
     */
    float m_grav_sw = 5.0f;
    float d_grav_sw = 0.0f;

    /* Stance/swing blending */
    float m_grf_u = 30.0f;
    float d_grf_u = 0.4f; /*!< normalized body weight */

    /* Safety limits */
    float torque_min_Nm = -20.0f;
    float torque_max_Nm = 20.0f;
    float extension_slew_rate_Nmps = 100.0f;

    /* Optional internal acceleration estimator */
    float torque_lpf_alpha = 0.03093f; /*!< EMA alpha for torque LPF at 1 kHz, fc approximately 5 Hz */
    float theta_k_ddot_lpf_alpha = 0.01867f; /*!< EMA alpha for knee acceleration LPF at 1 kHz, fc approximately 3 Hz */
    float theta_k_ddot_lpf_alpha_imu = 0.09f; /*!< Equivalent knee acceleration LPF alpha at 200 Hz */
  };

  enum class LeadingLeg : uint8_t
  {
    kUnknown = 0,
    kLeft,
    kRight,
  };

  struct BiLegContex
  {
    float dt_s = 0.001f;
    float delta_ajc_y_l_minus_r_cm = 0.0f;
    float delta_ajc_dist_cm = 0.0f;
    float delta_ajc_dist_fs_cm = 0.0f;
    float theta_k_abs_diff_rad = 0.0f;
    float left_grf_BW = 0.0f;
    float right_grf_BW = 0.0f;
    float left_heel_BW = 0.0f;
    float right_heel_BW = 0.0f;

    float gate_F_heel_left = 0.0f;
    float gate_F_heel_right = 0.0f;
    float gate_F_grf_inv_left = 0.0f;
    float gate_F_grf_inv_right = 0.0f;
    float gate_delta_ajc_dist = 0.0f;
    float gate_delta_ajc_dist_fs = 0.0f;

    /* ZZZ: STS */
    float gate_sts_knee_pos_sym = 0.0f;
    float gate_sts_left_foot_loaded = 0.0f;
    float gate_sts_right_foot_loaded = 0.0f;
    float gate_sts_both_foot_loaded = 0.0f;
    float gate_sts_left_knee_raise_vel = 0.0f;
    float gate_sts_right_knee_raise_vel = 0.0f;
    float gate_sts_bilateral_knee_raise_vel = 0.0f;
    float gate_sts_left_knee_lower_vel = 0.0f;
    float gate_sts_right_knee_lower_vel = 0.0f;
    float gate_sts_bilateral_knee_lower_vel = 0.0f;
    float bilateral_lower_vel_radps = 0.0f;
    float gate_sts_deep_flex_decay = 0.0f;
    float gate_sts_left_thigh_flex = 0.0f;
    float gate_sts_right_thigh_flex = 0.0f;
    float gate_sts_bilateral_thigh_flex = 0.0f;

    bool left_fs_accepted = false;
    bool right_fs_accepted = false;
    LeadingLeg leading_leg = LeadingLeg::kUnknown;
    uint64_t update_prev_us = 0u;
  };

  static DivekarParams divekar_params_;
  static BiLegContex bi_leg_ctx_;

  struct DivekarState
  {
    /* Segment and leg angles */
    float theta_trunk_rad = 0.0f; /*!< ZZZ: body global sagittal angle, for sts */
    float theta_th_rad = 0.0f; /*!< thigh global sagittal angle */
    float theta_sh_rad = 0.0f; /*!< shank global sagittal angle */
    float theta_la_rad = 0.0f; /*!< hip-ankle leg angle */
    float ankle_x_m = 0.0f; /*!< ankle X in hip frame */
    float ankle_y_m = 0.0f; /*!< ankle Y in hip frame */

    /* Knee kinematics */
    float theta_k_rad = 0.0f; /*!< knee flexion positive */
    float theta_k_dot_radps = 0.0f;
    float theta_kd_rad = 0.0f; /*!< θ_k - θ_k_hs, 预处理 */

    /* Task variables, normally frozen at latest heel strike */
    float delta_ajc_y_fs_cm = 0.0f; /*!< default unit: cm */

    /* ——— sigmoid gates (debug) ——— */
    float gate_theta_la = 0.0f; /*!< σ(θ_la), gates τ_a/τ_na */
    float gate_theta_kd_max = 0.0f; /*!< σ(θ_kd^max), tapers k_na */
    float gate_theta_k_LL_nested = 0.0f; /*!< x1·σ(θ_k)+x2, LL sigmoid offset */
    float gate_theta_k_dot_LL = 0.0f; /*!< σ(θ̇_k, offset), LL velocity taper */
    float gate_delta_ajc_y_a_fs = 0.0f; /*!< σ(δ_ajc_y, m=5), ascent modulation */
    float gate_delta_ajc_y_na_fs = 0.0f; /*!< σ(δ_ajc_y, m=-10), non-ascent modulation */

    float gate_theta_k_dot_sw = 0.0f; /*!< σ(θ̇_k, m=5), swing gravity gate */
    float gate_F_grf_u = 0.0f; /*!< σ(F_grf^ipsi, m=30), α-blend */

    float gate_sts_sit2stand = 0.0f;
    float gate_sts_stand2sit = 0.0f;
    float gate_sts_ctx = 0.0f;

    /* from states */
    float theta_k_hs_rad = 0.0f;
    float theta_kd_max_rad = 0.0f;
    float theta_k_dot_prev_radps = 0.0f;
    float theta_k_ddot_lpf_radps2 = 0.0f;
    uint32_t theta_k_dot_sample_id = 0u;
    float theta_k_dot_sample_elapsed_s = 0.0f;
    float tau_prev_Nm = 0.0f;
    float tau_divekar_lpf_prev_Nm = 0.0f;
    bool theta_k_dot_history_valid = false;
    bool torque_history_valid = false;
  } divekar_state_;

  struct DivekarOutput
  {
    float tau_sit2stand_Nm = 0.0f;
    float tau_stand2sit_Nm = 0.0f;
    float tau_sts_Nm = 0.0f;
    float tau_sts_mod_Nm = 0.0f;

    /* ——— torques in stance ——— */
    float tau_a_Nm = 0.0f; /*!< ascent spring torque */
    float tau_na_Nm = 0.0f; /*!< non-ascent spring-damper torque */
    float tau_LL_Nm = 0.0f; /*!< lifting-lowering spring torque */
    float tau_grav_st_Nm = 0.0f; /*!< gravity compensation torque in stance */

    /* ——— task-sensitization torques in stance ——— */
    float tau_a_mod_Nm = 0.0f;
    float tau_na_mod_Nm = 0.0f;
    float tau_LL_mod_Nm = 0.0f;
    float tau_grav_st_mod_Nm = 0.0f;
    float tau_st_Nm = 0.0f;

    /* ——— torques in swing ——— */
    float tau_grav_sw_Nm = 0.0f;
    float tau_inertial_sw_Nm = 0.0f;
    float tau_sd_sw_Nm = 0.0f;
    float tau_sw_Nm = 0.0f;

    /* ——— output torques ——— */
    float tau_divekar_Nm = 0.0f; /*!< blended torque, before filtering */
    float tau_divekar_lpf_Nm = 0.0f; /*!< 5Hz LPF filtered */
    float tau_divekar_lpf_limited_Nm = 0.0f; /*!< after safety limits */

    /* ——— equivalent MotionControl output in joint coordinates ——— */
    float mc_stiffness_Nm_per_rad = 0.0f;
    float mc_damping_Nm_s_per_rad = 0.0f;
    float mc_position_ref_rad = 0.0f;
    float mc_velocity_ref_radps = 0.0f;
    float mc_torque_feedforward_Nm = 0.0f;
  } divekar_output_;

  void ComputeAnkleGeometry();
  void DivekarReset();
  void DivekarUpdate();
  static void ResetDivekarBiLegContext();
  static void UpdateDivekarBiLegContext(KneeJoint &left_knee, KneeJoint &right_knee, const ExoData &pe);
  static void LatchDivekarLeadingLeg(KneeJoint &left_knee, KneeJoint &right_knee, bool left_fs, bool right_fs);

private:
  CtrlMode ctrl_mode_ = CtrlMode::kOpenLoopTorque;
  uint32_t force_ramp_ms_ = 0; /*!< 模式切换后力矩爬升计时 (ms) */
  static constexpr uint32_t kForceRampDurationMs = 300; /*!< 力矩从 0→1 的爬升时长 */

  static float Step(float x)
  {
    return (x > 0.0f) ? 1.0f : 0.0f;
  }

  static float Sigmoid(float x, float m, float d)
  {
    const float z = -m * (x - d);
    if (z > 60.0f) return 0.0f;
    if (z < -60.0f) return 1.0f;
    return 1.0f / (1.0f + expf(z));
  }

  void UpdateDivekarFeedbackKinematics();
  void UpdateDivekarMotionControlOutput();
  float GetThetaKddotLpf(float theta_k_dot_radps, float dt_s, bool has_new_sample, float alpha);
  float GetTorqueLpf(float tau_unfiltered_Nm);
  float ApplySafetyLimits(float tau_unfiltered_Nm, float dt_s);
};

/* ============================================================================
 * 8. Estimators
 * ========================================================================== */

/**
 * @brief FSR 步态相位估计器
 *
 * 处理流程:
 *   1. UpdateContactAdaptive(): state-constrained contact detection from raw FSR readings
 *   3. PrepareUpdate()/FinalizeUpdate()/CommitUpdate(): 双侧同步提取 FS/FO 与局部接触事件,
 *      维护事件间期滑动平均窗口, 计算步态相位百分比
 *
 * @note  percent_gait_ 从 0 (FS) 到 100 (下一次 FS),
 *        基于 expected_gait_duration_ms_ 线性插值, 异常值通过窗口系数过滤
 */
class FsrGaitEstimator
{
public:
  explicit FsrGaitEstimator(FsrGaitData &gait_data, ExoData &pe) :
    gait_data_(gait_data),
    pe_(pe)
  {
  }
  virtual ~FsrGaitEstimator() = default;

  void PrepareUpdate(uint32_t now_ms);
  void CheckDataFreshness(uint32_t now_ms); /*!< 每周期检查数据超时, 直接更新 is_data_fresh_ */
  void FinalizeUpdate(uint32_t now_ms);
  void CommitUpdate();
  void Reset();
  void ResetContact(); /*!< 重置自适应接触检测状态 */

private:
  void UpdateContactAdaptive(FsrSensorData &sensor, bool allow_baseline_update);
  void FinalizeStepPeak(FsrSensorData &sensor);
  static void ResetSensorContactState(FsrSensorData &sensor);

  void ClearCycleEvents();
  void DetectOwnFsrEvents();
  void ApplyEventGuards(uint32_t now_ms);
  void UpdateEventTimings(uint32_t now_ms);
  void ResolvePhase();
  void UpdatePercentages(uint32_t now_ms);
  void UpdateValidity();
  void UpdateExpectedDuration(uint32_t duration_ms, uint32_t duration_window_ms[], float &expected_duration_ms);
  void RecordEventTimestamp(uint8_t ev_idx, uint32_t now_ms);

  uint32_t last_contact_sample_ms_ = 0u; /*!< 最近处理的 FSR 样本时间戳 */

  static constexpr uint32_t kMinStanceDurationMs = 150u;
  static constexpr uint32_t kMinSwingDurationMs = 150u;
  static constexpr uint32_t kMinStepDurationMs = 350u;
  static constexpr uint32_t kMaxStepDurationMs = 4000u;
  static constexpr uint32_t kMinLocalContactEventIntervalMs = 50u;

  FsrGaitData &gait_data_;
  ExoData &pe_; /*!< 全局数据中心, 通过 gait_data_.is_left_ 选择左/右侧数据 */
};

/* TODO */
class StairPhaseEstimator
{
public:
  explicit StairPhaseEstimator(ExoData &pe, SideData &ps) :
    pe_(pe),
    ps_(ps)
  {
  }
  virtual ~StairPhaseEstimator() = default;

  void Update();
  void Reset();

private:
  void TransitionTo(StairPhase next_phase);

  ExoData &pe_;
  SideData &ps_;

  static constexpr float kStairStepContactAngleThresh = 40.0f; /*!< 上楼触台阶时的最小膝角 (deg) */
  static constexpr float kPullUpStartDeltaDeg = 10.0f; /*!< 触地后膝角下降超过该值则进入拉升 (deg) */
  static constexpr float kPullUpStartVelThresh = -45.0f; /*!< 上楼触地后明显伸膝则提前进入拉升 (deg/s) */
  static constexpr float kPullUpStartVelMinDeltaDeg = 2.0f; /*!< 速度触发拉升前所需的最小伸膝角度变化 (deg) */
  static constexpr float kStairDescentOppositeKneeAngleThresh = 20.0f; /*!< 下楼本侧触地时对侧支撑膝角阈值, 用于排除平地步态 (deg) */
  static constexpr float kControlledLoweringStartDeltaDeg = 12.0f; /*!< 下楼触地后膝角增大超过该值则进入受控下降 (deg) */
  static constexpr float kControlledLoweringVelMinDeltaDeg = 3.0f; /*!< 速度触发受控下降前所需的最小屈膝角度变化 (deg) */
  static constexpr float kKneeFlexingVelThresh = 45.0f; /*!< 下楼屈膝角速度辅助阈值 (deg/s) */
  static constexpr float kKneeStraightAngle = 15.0f; /*!< 接近伸直的角度 (deg) */
  static constexpr float kForwardEndFlexionDeltaDeg = 20.0f; /*!< 前向过渡中二次屈膝兜底结束阈值 (deg) */
};

class StsPhaseEstimator
{
public:
  explicit StsPhaseEstimator(ExoData &pe) :
    pe_(pe)
  {
  }
  virtual ~StsPhaseEstimator() = default;

  void Update();
  void Reset();

private:
  void TransitionTo(StsPhase next_phase);

  ExoData &pe_;

  static constexpr float kKneeExtendingVelThresh = -5.0f; /*!< 起立伸膝角速度阈值 (deg/s) */
  static constexpr float kKneeStraightAngle = 15.0f; /*!< 接近伸膝完成的膝角 (deg) */
  static constexpr float kSeatedKneeAngle = 65.0f; /*!< 无坐姿参考时判定接近坐姿的膝屈曲角 (deg) */
  static constexpr float kKneeStillVelThresh = 8.0f; /*!< 坐下完成时膝角速度接近静止阈值 (deg/s) */
  static constexpr float kCompleteThreshDeg = 6.0f; /*!< 大腿角度接近坐/站参考的完成阈值 (deg) */
  static constexpr float kMinTransitionRangeDeg = 10.0f; /*!< 坐姿/站姿参考最小角度差保护 (deg) */
};

class AdaptiveOscillator
{
public:
  explicit AdaptiveOscillator(ExoData &pe) :
    pe_(pe)
  {
  }
  virtual ~AdaptiveOscillator() = default;

  void Update();
  void Reset();

private:
  void ResetCoreState();
  static bool IsAoImuUsable(const ImuData &imu);
  static bool IsAoFsrUsable(const FsrGaitData &fsr);
  static float ClampValue(float value, float lower, float upper);
  static float WrapTo2Pi(float angle_rad);
  static float WrapToPi(float angle_rad);

  ExoData &pe_;

  static constexpr uint64_t kMaxTstrideUs = 3.0 * 1000000;
  static constexpr uint64_t kMinTstrideUs = 0.1 * 1000000;
  static constexpr uint64_t kMaxStoppingDurationUs = 0.5 * 1000000;
  static constexpr float kEmaTauS = 0.2f;
  static constexpr uint8_t kNumAOs = 3;
  static constexpr float kDefaultFrequencyHz = 1.0f;
  static constexpr float kMinFrequencyHz = 0.35f;
  static constexpr float kMaxFrequencyHz = 2.50f;
  static constexpr float kMinOscAmplitudeRad = 0.05f;
  static constexpr float kInitialHarmonicAmplitudeRad = 0.10f;
  static constexpr float kMaxHarmonicAmplitudeRad = 2.0f;
  static constexpr float kMaxBiasRad = 1.5f;
  static constexpr float kMaxNormalizedError = 2.0f;

  uint64_t tprev_sys_us_ = 0;
  uint64_t left_tk_sys_us_ = 0;
  uint64_t right_tk_sys_us_ = 0;
  uint64_t total_foot_contact_duration_us_ = 0;
  bool contact_state_initialized_ = false;
  bool left_contact_prev_ = false;
  bool right_contact_prev_ = false;

  float v_phi_ = 10.0f;
  float v_omega_ = 6.0f;
  float eta_ = 1.0f;
  float kp_ = 1.0f;

  float hat_x_ = 0.0f;
  float omega_ = _2PI * kDefaultFrequencyHz;
  float phi_[kNumAOs] = {0.0f};
  float alpha_[kNumAOs] = {kInitialHarmonicAmplitudeRad};
  float alpha0_ = 0.0f;
  float vel_energy_ema_ = 0.0f;
  float left_epsilon_phi_tk_ = 0.0f;
  float right_epsilon_phi_tk_ = 0.0f;
  float left_phi_e_ = 0.0f;
  float right_phi_e_ = 0.0f;
};

/* ============================================================================
 * 9. Side Aggregation
 * ========================================================================== */

class Side
{
public:
  explicit Side(bool is_left,
                ExoData &pe,
                DjiEscHub &dji_esc_hub,
                FDCAN_HandleTypeDef &motor_can,
                UART_HandleTypeDef &huart) :
    pe_(pe),
    ps_(is_left ? pe_.left_side_ : pe_.right_side_),
    hip_joint_(pe,
               ps_,
               ps_.hip_joint_,
               motor_can,
               is_left ? ExoJointCanID::kLeftHip : ExoJointCanID::kRightHip),
    knee_joint_(is_left, pe, motor_can),
    knee_sea_joint_(pe,
                    ps_,
                    ps_.knee_sea_joint_,
                    dji_esc_hub,
                    is_left ? DjiEsc::EscId::kId1 : DjiEsc::EscId::kId2,
                    huart),
    ankle_joint_(is_left, pe, motor_can),
    fsr_gait_estimator_(ps_.fsr_gait_data_, pe),
    stair_phase_estimator_(pe, ps_)
  {
  }

  virtual ~Side() = default;
  void CaptureStandPosture();
  void ClearStandPosture();
  void UpdateCalibrationStatus();
  void Read();
  void Assist();
  bool IsMotorConnect();
  void Standby();
  void Shutdown();

  ExoData &pe_;
  SideData &ps_;
  HipJoint hip_joint_;
  KneeJoint knee_joint_;
  KneeSeaJoint knee_sea_joint_;
  AnkleJoint ankle_joint_;
  FsrGaitEstimator fsr_gait_estimator_;
  StairPhaseEstimator stair_phase_estimator_;
};

/* ============================================================================
 * 10. Shell And IMU Hubs
 * ========================================================================== */

class ExoShell : public Shell
{
public:
  explicit ExoShell(UART_HandleTypeDef &huart, Exo &exo);
  ~ExoShell() = default;

  void OnCmdSetLocoMode(int argc, char **argv);
  void OnCmdSetSlope(int argc, char **argv);

private:
  static bool StringEquals(const char *lhs, const char *rhs);
  static const char *LocoModeToString(LocoMode mode);
  static bool ParseLocoMode(const char *text, LocoMode &mode);

  Exo &exo_;
};

/**
 * @brief 躯干 BMI088 IMU 姿态解算模块
 * @note  使用 Mahony AHRS 滤波器 (6轴: 陀螺仪 + 加速度计),
 *        采样频率标称 1000Hz (需根据实际 TIM 配置调整)
 */
class BodyImu
{
public:
  explicit BodyImu(ExoData &pe) :
    pe_(pe),
    body_imu_(pe.body_imu_)
  {
  }
  virtual ~BodyImu() = default;

  void Read();

  ExoData &pe_;
  ImuData &body_imu_;
  Mahony mahony_filter_{1000.0f};
};

class DaMiaoImuHub
{
public:
  enum CanID : uint8_t
  {
    kLeftShank = 0x01,
    kRightShank = 0x02,
    kLeftThigh = 0x03,
    kRightThigh = 0x04
  };
  enum MstID : uint8_t
  {
    kLeftShankMst = 0x11,
    kRightShankMst = 0x12,
    kLeftThighMst = 0x13,
    kRightThighMst = 0x14
  };

  explicit DaMiaoImuHub(ExoData &pe, FDCAN_HandleTypeDef &hfdcan) :
    pe_(pe),
    hfdcan_(hfdcan)
  {
  }
  virtual ~DaMiaoImuHub() = default;

  void CanRxCallback(uint32_t can_id, const uint8_t *data, uint32_t dlc);

  ExoData &pe_;
  FDCAN_HandleTypeDef &hfdcan_;

private:
  static ImuData &ImuById(ExoData &pe, uint8_t id)
  {
    switch (id)
    {
    case kLeftShank:
      return pe.left_side_.shank_imu_;
    case kRightShank:
      return pe.right_side_.shank_imu_;
    case kLeftThigh:
      return pe.left_side_.thigh_imu_;
    case kRightThigh:
    default:
      return pe.right_side_.thigh_imu_;
    }
  }

  static float UintToFloat(uint32_t x, float x_min, float x_max, int num_bits)
  {
    uint32_t span = (1 << num_bits) - 1;
    float offset = x_max - x_min;
    return offset * x / span + x_min;
  }

  static constexpr float kAccelCanMax = 235.2f;
  static constexpr float kAccelCanMin = -235.2f;
  static constexpr float kGyroCanMax = 34.88f;
  static constexpr float kGyroCanMin = -34.88f;
  static constexpr float kPitchCanMax = 90.0f;
  static constexpr float kPitchCanMin = -90.0f;
  static constexpr float kRollCanMax = 180.0f;
  static constexpr float kRollCanMin = -180.0f;
  static constexpr float kYawCanMax = 180.0f;
  static constexpr float kYawCanMin = -180.0f;
  static constexpr float kTempMin = 0.0f;
  static constexpr float kTempMax = 60.0f;
  static constexpr float kQuaternionMin = -1.0f;
  static constexpr float kQuaternionMax = 1.0f;
};

class HiPnucImuHub
{
public:
  enum CanID : uint8_t
  {
    kLeftShank = 0x01,
    kRightShank = 0x02,
    kLeftThigh = 0x03,
    kRightThigh = 0x04
  };

  explicit HiPnucImuHub(ExoData &pe, FDCAN_HandleTypeDef &hfdcan) :
    pe_(pe),
    hfdcan_(hfdcan)
  {
  }
  virtual ~HiPnucImuHub() = default;

  void CanRxCallback(uint32_t can_id, const uint8_t *data, uint32_t dlc);

  ExoData &pe_;
  FDCAN_HandleTypeDef &hfdcan_;

private:
  static ImuData &ImuById(ExoData &pe, uint8_t id)
  {
    switch (id)
    {
    case kLeftShank:
      return pe.left_side_.shank_imu_;
    case kRightShank:
      return pe.right_side_.shank_imu_;
    case kLeftThigh:
      return pe.left_side_.thigh_imu_;
    case kRightThigh:
    default:
      return pe.right_side_.thigh_imu_;
    }
  }

  static inline int16_t unpack_i16(const uint8_t *d)
  {
    return (int16_t)(d[0] | (d[1] << 8));
  }

  static constexpr float kAccelMGtoMps2 = 0.00980665f;
  static constexpr float kGyro01DpsToRps = 0.00174533f;
  static constexpr float kEuler001DegToRad = 0.000174533f;
  static constexpr float kQuatScale = 1.0f / 10000.0f;
};

/* ============================================================================
 * 11. Intention Recognition
 * ========================================================================== */

struct SvmFeatureVector
{
  float max_thigh_pitch_rad;
  float max_knee_flexion_rad;
  float mean_thigh_omega_radps;
  float var_accel_z_mps2;
  int zero_crossings;
  float final_calf_pitch_rad;
};

struct SvmPrediction
{
  LocoMode mode = LocoMode::kWalking;
  float confidence = 0.0f;
  bool is_valid = false; /*!< false 表示 SVM 本次放弃判断, 保持当前模式 */
};

class StreamingFeatureExtractor
{
public:
  explicit StreamingFeatureExtractor(ExoData &pe) :
    pe_(pe)
  {
  }
  virtual ~StreamingFeatureExtractor() = default;

  void Update(float thigh_pitch, float calf_pitch, float thigh_omega, float knee_omega, float accel_z)
  {
    sample_count_++;

    if (thigh_pitch > max_thigh_pitch_rad_) max_thigh_pitch_rad_ = thigh_pitch;

    float knee_flexion = thigh_pitch - calf_pitch;
    if (knee_flexion > max_knee_flexion_rad_) max_knee_flexion_rad_ = knee_flexion;

    sum_thigh_omega_radps_ += thigh_omega;
    sum_accel_z_mps2_ += accel_z;
    sum_sq_accel_z_mps2_ += (accel_z * accel_z);

    if (sample_count_ > 1 && (knee_omega * last_knee_omega_radps_ < 0.0f))
    {
      zero_crossings_++;
    }
    last_knee_omega_radps_ = knee_omega;
  }

  SvmFeatureVector GetFeatures(float current_calf_pitch)
  {
    SvmFeatureVector fv = {0};
    if (sample_count_ == 0) return fv;

    fv.max_thigh_pitch_rad = max_thigh_pitch_rad_;
    fv.max_knee_flexion_rad = max_knee_flexion_rad_;
    fv.zero_crossings = zero_crossings_;
    fv.final_calf_pitch_rad = current_calf_pitch;

    fv.mean_thigh_omega_radps = sum_thigh_omega_radps_ / sample_count_;
    float mean_accel = sum_accel_z_mps2_ / sample_count_;
    fv.var_accel_z_mps2 = (sum_sq_accel_z_mps2_ / sample_count_) - (mean_accel * mean_accel);
    if (fv.var_accel_z_mps2 < 0.0f) fv.var_accel_z_mps2 = 0.0f;  // 防浮点溢出

    return fv;
  }

  void Reset()
  {
    sample_count_ = 0;
    max_thigh_pitch_rad_ = 0.0f;
    max_knee_flexion_rad_ = 0.0f;
    sum_thigh_omega_radps_ = 0.0f;
    sum_accel_z_mps2_ = 0.0f;
    sum_sq_accel_z_mps2_ = 0.0f;
    last_knee_omega_radps_ = 0.0f;
    zero_crossings_ = 0;
  }

private:
  ExoData &pe_;

  int sample_count_ = 0;
  float max_thigh_pitch_rad_ = 0.0f;
  float max_knee_flexion_rad_ = 0.0f;
  float sum_thigh_omega_radps_ = 0.0f;
  float sum_accel_z_mps2_ = 0.0f;
  float sum_sq_accel_z_mps2_ = 0.0f;
  float last_knee_omega_radps_ = 0.0f;
  int zero_crossings_ = 0;
};

class IntentionRecognizer
{
public:
  explicit IntentionRecognizer(ExoData &pe) :
    pe_(pe),
    left_feature_extractor_(pe),
    right_feature_extractor_(pe)
  {
  }
  virtual ~IntentionRecognizer() = default;

  void Update(); /*!< 主入口 */

  /* ——— 可调参数 (Shell 可读写) ——————————————————————————————— */
  struct ArbiterOverride
  {
    LocoMode forced_locomode = LocoMode::kWalking;
    bool enable_locomode_override = true;
  } override_usr_;

  float kSlopeGain = 1.0f; /*!< SagittalFromStandRefDeg -> 地形坡度 (deg) 线性增益 */

private:
  ExoData &pe_;
  struct SlopeSideState
  {
    bool was_full_contact = false;
    uint32_t contact_duration_ms = 0;
    uint32_t sample_count = 0;
    float foot_pitch_sum_deg = 0.0f;
    float last_step_slope_deg = 0.0f;
    bool has_step_slope = false;
    float smooth_window_deg[3] = {};
    uint8_t smooth_window_count = 0;
    uint8_t smooth_window_idx = 0;
    uint32_t last_sample_ms = 0;
  };
  SlopeSideState left_slope_state_, right_slope_state_;
  uint32_t last_slope_sample_ms_ = 0;

  static constexpr uint8_t kSlopeSmoothWindowSteps = 2u;
  static constexpr uint32_t kSlopeMinFullContactMs = 30u;
  static constexpr float kSlopeEnterDeg = 5.0f;
  static constexpr float kSlopeExitDeg = 3.0f;
  static constexpr float kSlopeSideDisagreeDeg = 5.0f;
  static constexpr float kSlopeConfidenceValidThresh = 0.45f;

  void UpdateSlopeEstimate(uint32_t now_ms, uint32_t delta_ms);
  void UpdateRampModeBySlope(uint32_t now_ms);

  /* ——— SVM 地形分类 ——————————————————————————————————————————— */
  StreamingFeatureExtractor left_feature_extractor_;
  StreamingFeatureExtractor right_feature_extractor_;
  bool left_svm_done_this_step_ = false;
  bool right_svm_done_this_step_ = false;

  static constexpr int kVoteBufferSize = 3;
  LocoMode vote_buffer_[kVoteBufferSize] = {LocoMode::kWalking};
  float vote_confidence_[kVoteBufferSize] = {};
  int vote_buffer_idx_ = 0;
  uint8_t vote_count_ = 0;
  static constexpr float kSvmConfidenceThreshold = 0.20f;

  bool enable_svm_router_ = false; /*!< 启用 SVM 地形分类投票, 默认关 */
  void UpdateGaitSvmRouter();
  void PushVote(const SvmPrediction &prediction);
  SvmPrediction GetMajorityVote();
  SvmPrediction SvmClassifyIntention(const SvmFeatureVector &fv);

  /* ——— 静态规则检测 ——————————————————————————————————————————— */
  static constexpr uint32_t kDoubleSupportTimeoutMs = 600;
  static constexpr float kStillEnergyThreshold = 15.0f;
  static constexpr float kSitLeanVelThresh = -25.0f;
  static constexpr float kKneeBendVelThresh = 15.0f;
  static constexpr float kKneeBendAngleThresh = 15.0f;
  static constexpr float kKneeExtendedAngleThresh = 20.0f; /*!< 双支撑看门狗: 膝伸直判定阈值 (deg) */

  void UpdateRuleBasedDetector(uint32_t now_ms, uint32_t delta_ms);

  /* ——— 通用辅助 ———————————————————————————————————————————————— */
  uint32_t last_update_ms_ = 0;
  uint32_t last_mode_change_ms_ = 0;
  static constexpr uint32_t kMinModeHoldMs = 350;
  bool TryAcceptMode(LocoMode next_mode, uint32_t now_ms, bool force = false);
  bool IsLegalTransition(LocoMode from, LocoMode to) const;
  static bool IsGaitMode(LocoMode mode);
  static bool IsSvmGaitMode(LocoMode mode);
};

/* ============================================================================
 * 12. Top-Level Exo Controller
 * ========================================================================== */
class Exo
{
public:
  explicit Exo(ExoData &pe, ExoHardware &hw) :
    pe_(pe),
    hw_(hw),
    dji_esc_hub_(hw.motor_can1),
    dm_imu_hub_(pe, hw.sensor_can),
    hipnuc_imu_hub_(pe, hw.sensor_can),
    ao_(pe),
    intention_recognizer_(pe),
    sts_phase_estimator_(pe),
    body_imu_(pe),
    shell_(hw.shell_uart, *this),
    left_side_(true, pe, dji_esc_hub_, hw.motor_can1, hw.left_mag_encoder_uart),
    right_side_(false, pe, dji_esc_hub_, hw.motor_can2, hw.right_mag_encoder_uart)
  {
  }
  ~Exo() = default;

  void Initialize();
  void Run();
  void Read();
  void Calibrate();
  void ResetCalibration();
  void ResetEstimations();
  void ResetControllerStates();

  void ClearStandPosture();
  void CaptureStandPosture();

  void Estimate();
  void Standby();
  void Assist();
  void Shutdown();

  void CheckSystemHealth();
  void VofaSendTelemetry();
  bool IsMotorConnect();
  bool IsCalibrateDone();
  bool IsStopWalking();

  void CanRxCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t can_id, const uint8_t *data, uint32_t dlc);
  void SensorUartReceiveDma(void);
  void UartRxCallback(UART_HandleTypeDef *huart, uint16_t data_size);
  void UartErrorCallback(UART_HandleTypeDef *huart);

  void SpiRxStart(void);
  void SpiRxCallback(void);
  void SpiErrorCallback(SPI_HandleTypeDef *hspi);
  void ProcessSpiData(void);

  ExoData &pe_;
  ExoHardware &hw_;

  DjiEscHub dji_esc_hub_;
  DaMiaoImuHub dm_imu_hub_;
  HiPnucImuHub hipnuc_imu_hub_;
  AdaptiveOscillator ao_;
  IntentionRecognizer intention_recognizer_;
  StsPhaseEstimator sts_phase_estimator_;
  BodyImu body_imu_;
  ExoShell shell_;
  StateLed state_led_;
  ChirpGenerator chirp_generator_{1.0f, 8.0f, 60.0f};
  Side left_side_;
  Side right_side_;

  volatile bool spi_data_ready_ = false;
  volatile uint8_t spi_dma_readed_size_ = 0;
  volatile uint8_t spi_dma_reading_idx_ = 0;
  volatile uint8_t spi_dma_handling_idx_ = 1;

private:
  void VofaSendSensorTelemetry(uint32_t now_ms, uint32_t loop_cnt);
  void VofaSendStairTelemetry(uint32_t now_ms, uint32_t loop_cnt);

  void UpdateHumanJointKinematics();
  void SensorUartRxCallback(const uint8_t *data, uint16_t data_size);
  static ExoData::SysEvent AllowedEventsForState(ExoData::State s);
  static inline void ClearNonCriticalEvents(ExoData &pe)
  {
    pe.pending_events_ &= ~ExoData::SysEvent::kWakeup;
    pe.pending_events_ &= ~ExoData::SysEvent::kStartCalibrate;
    pe.pending_events_ &= ~ExoData::SysEvent::kStartAssist;
    pe.pending_events_ &= ~ExoData::SysEvent::kStopAssist;
    pe.pending_events_ &= ~ExoData::SysEvent::kEnterSleep;
  }
};

#endif
