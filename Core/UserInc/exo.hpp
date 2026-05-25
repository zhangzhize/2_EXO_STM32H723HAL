/**
 * @file exo.hpp
 * @author zzz
 * @brief 
 * @version 0.1
 * @date 2025-12-15
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef EXO_HPP
#define EXO_HPP

#include <cstdint>
#include "utils.h"
#include "status_led.hpp"
#include "robstride.hpp"
#include "dm_motor.hpp"
#include "dji_esc.hpp"
#include "mag_encoder.hpp"
#include "force_profile_generator.hpp"
#include "pid.hpp"
#include "disturbance_observer.hpp"
#include "shell.hpp"
#include "chirp_generator.hpp"
#include "mahony.hpp"

/** 应该与NRF54代码中的一致 */
typedef struct __attribute__((packed)) foot_sensor_packet_t
{
    int32_t mV_heel;
    int32_t mV_toe;
    float mV_pull;
    float quatI;
    float quatJ;
    float quatK;
    float quatReal;
} foot_sensor_packet_t;

typedef struct __attribute__((packed)) exo_sensor_packet_t
{
    foot_sensor_packet_t left_foot;
    foot_sensor_packet_t right_foot;
} exo_sensor_packet_t;


/** for bitwise operations */
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

/** Forward declarations */
class IMUData;
class JointData;
class AnkleData;
class KneeSeaJointData;
class FsrGaitData;
class SideData;
class ExoData;

class AnkleJoint;
class KneeJoint;
class KneeSeaJoint;
class FsrGaitEstimator;
class AdaptiveOscillator;
class Side;
class ExoShell;
class BodyImu;
class Exo;

class ImuData
{
public:
    explicit ImuData(bool is_left = true) : is_left_(is_left) {}
    virtual ~ImuData() = default;

    float q_[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // real, i, j, k
    float roll_rad_ = 0.0f;
    float pitch_rad_ = 0.0f;
    float yaw_rad_ = 0.0f;
    float roll_deg_ = 0.0f;
    float pitch_deg_ = 0.0f;
    float yaw_deg_ = 0.0f;

    float accel_mps2_[3] = {0.0f, 0.0f, 0.0f};
    float gyro_degps_[3] = {0.0f, 0.0f, 0.0f};
    float magnet_uT_[3] = {0.0f, 0.0f, 0.0f};
    float chip_temp_c_ = 0.0f;

    bool is_left_;
    bool is_used_ = true;
    bool is_calibrated_ = false;
};

class JointData
{
public:
    explicit JointData(bool is_left = true) : is_left_(is_left) {}
    virtual ~JointData() = default;

    float pos_ref_rad_ = 0.0;       /** 人体关节角度参考 */
    float pos_rad_ = 0.0f;          /** 人体关节角度反馈 */
    float pos_offset_rad_ = 0.0f;   /** 人体关节角度偏移, 用于标定 */
    float vel_ref_radps_ = 0.0f;    /** 人体关节角速度参考 */
    float vel_radps_ = 0.0f;        /** 人体关节角速度反馈 */
    float tor_interact_ref_Nm_ = 0.0f;  /** 人机交互力矩参考 */
    float tor_interact_Nm_ = 0.0f;
    float tor_ref_Nm_ = 0.0f;       /** 还没想好用于表示什么力矩 */
    float tor_Nm_ = 0.0f;           /** 还没想好用于表示什么力矩 */

    bool is_left_;              /** 表示该数据是左侧关节的还是右侧的 */
    bool is_used_ = false;      /** 表示该关节是否使用 */
    bool is_calibrated_ = false;
};

class AnkleData : public JointData
{
public:
    explicit AnkleData(bool is_left = true) : JointData(is_left) {}
    virtual ~AnkleData() = default;

    float plantarflexion_force_N_ = 0.0f; 
};

class KneeSeaJointData : public JointData
{
public:
    explicit KneeSeaJointData(bool is_left = true) : JointData(is_left) {}
    virtual ~KneeSeaJointData() = default;

    float pos_slider_mm_ = 0.0f;    /** 滑块位移 */
    float pos_slider_offset_mm_ = 0.0f;  /** 滑块在最大膝伸展(暂且当作0度)时的位置 */
    float vel_slider_mmps_ = 0.0f; /** 滑块线速度 */
    float screw_lead_rad2mm_ = 2.0f/ _2PI;  /** 滑块从旋转到直线位移的系数 */

    float pos_linear_encoder_mm_ = 0.0f; /** 外框位移 */
    float pos_linear_encoder_offset_mm_ = 13587.649414f;  /** 外框在最大膝伸展(暂且当作0度)时的位置, 暂且当作0度 */
    float max_pos_linear_encoder_mm_ = 13636.950195f;   /** 当前最大膝弯曲(暂且当作90度)时的位置 */
    float vel_linear_encoder_mmps_ = 0.0f;   /** 外框线速度 */

    float pos_bias_mm_ = 0.0f;      /** 滑块与外框位移之差 */
    float force_spring_ref_N_ = 0.0f;  /** 弹簧参考力 */
    float force_spring_N_ = 0.0f;      /** 弹簧反馈力(位移差xspring_stiffness_Npmm_) */
    float spring_stiffness_Npmm_ = 2 * 15.637f;   /** 两根弹簧的"理论"刚度 */
};

struct FsrSensorData
{
    static constexpr uint32_t kCalibrationDurationMs = 5000u;
    static constexpr uint8_t kNumRefinementSteps = 7u;
    static constexpr float kSchmittLowerThresholdRefinement = 0.33f;
    static constexpr float kSchmittUpperThresholdRefinement = 0.66f;

    float raw_reading = 0.0f;
    float calibrated_reading = 0.0f;
    float calibration_min = 0.0f;
    float calibration_max = 0.0f;

    float step_max_sum = 0.0f;
    float step_max = 0.0f;
    float step_min_sum = 0.0f;
    float step_min = 0.0f;
    float calibration_refinement_min = 0.0f;
    float calibration_refinement_max = 0.0f;

    float schmitt_lower_threshold_calc_contact = 0.15f; //
    float schmitt_upper_threshold_calc_contact = 0.25f; //

    uint32_t calibration_start_sys_ms = 0;
    uint8_t refinement_step_count = 0;

    bool last_do_calibrate = false;
    bool last_do_refinement = false;
    bool ground_contact_during_refinement = false;
    bool ground_contact = false;
};

class FsrGaitData
{
public:
    explicit FsrGaitData(bool is_left = true) : is_left_(is_left) {}
    virtual ~FsrGaitData() = default;

    static constexpr uint8_t kNumStepsAvg = 3;

    FsrSensorData heel_;
    FsrSensorData toe_;

    uint32_t step_times_[kNumStepsAvg] = {0};
    uint32_t stance_times_[kNumStepsAvg] = {0};
    uint32_t swing_times_[kNumStepsAvg] = {0};

    uint32_t ground_strike_timestamp_ = 0;
    uint32_t prev_ground_strike_timestamp_ = 0;
    uint32_t toe_strike_timestamp_ = 0;
    uint32_t prev_toe_strike_timestamp_ = 0;
    uint32_t toe_off_timestamp_ = 0;
    uint32_t prev_toe_off_timestamp_ = 0;

    float percent_gait_ = -1.0F;
    float percent_stance_ = -1.0f;
    float percent_swing_ = -1.0f;
    float expected_step_duration_ = -1.0f;
    float expected_stance_duration_ = -1.0f;
    float expected_swing_duration_ = -1.0f;
    float expected_duration_window_upper_coeff_ = 1.75;
    float expected_duration_window_lower_coeff_ = 0.25f;

    bool ground_strike_ = false;
    bool toe_strike_ = false;
    bool toe_off_ = false;
    bool toe_on_ = false;
    bool heel_contact_state_ = false;
    bool toe_contact_state_ = false;
    bool prev_heel_contact_state_ = true;
    bool prev_toe_contact_state_ = true;

    bool do_calibration_toe_fsr_ = false;
    bool do_calibration_refinement_toe_fsr_ = false;
    bool do_calibration_heel_fsr_ = false;
    bool do_calibration_refinement_heel_fsr_ = false;

    bool is_left_;
    bool is_used_ = true;
    bool is_calibrated_ = false;
};

class AoData
{
public:
    explicit AoData() = default;
    virtual ~AoData() = default;

    uint32_t left_event_cnt_ = 0;
    uint32_t right_event_cnt_ = 0;
    float left_phi_comp_rad_ = 0.0f;
    float right_phi_comp_rad_ = 0.0f;

    bool is_used_ = true;
};

class SideData
{
public:
    explicit SideData(bool is_left = true) : hip_joint_(is_left), knee_joint_(is_left), knee_sea_joint_(is_left), ankle_joint_(is_left), fsr_gait_data_(is_left), foot_imu_(is_left), is_left_(is_left) {}
    virtual ~SideData() = default;

    JointData hip_joint_;
    JointData knee_joint_;
    KneeSeaJointData knee_sea_joint_;
    AnkleData ankle_joint_;
    FsrGaitData fsr_gait_data_;
    ImuData foot_imu_;
    ImuData shank_imu_;
    ImuData thigh_imu_;

    bool is_left_;
    bool is_used_ = true;
    bool is_calibrated_ = false;
};

class ExoData
{
public:
    /*  9 RGB Color Codes
    const uint8_t kRGBColors[9][3] = {
        {0x00, 0x00, 0x00}, // Off       #000000
        {0x00, 0xFF, 0x00}, // Green     #00FF00
        {0x00, 0x00, 0xFF}, // Blue      #0000FF
        {0xFF, 0xFF, 0x00}, // Yellow    #FFFF00
        {0xFF, 0x00, 0xFF}, // Magenta   #FF00FF
        {0x00, 0xFF, 0xFF}, // Cyan      #00FFFF
        {0xFF, 0x8C, 0x00}, // Orange    #FF8C00
        {0x80, 0x00, 0x80}, // Purple    #800080
        {0xFF, 0x00, 0x00}, // Red       #FF0000
    };
    */
    enum class State : uint8_t
    {
        kSleep = 0U,
        kWaitMotorComm,
        kCalibrating,
        kReady,
        kAssisting,
        kFaultLowBattery,
        kFaultSystem,
    };

    enum class Error : uint32_t
    {
        kNone            = 0 << 0,
        kBatteryLow      = 1 << 0,
        kLeftHipFault    = 1 << 1,
        kRightHipFault   = 1 << 2,
        kLeftKneeFault   = 1 << 3,
        kRightKneeFault  = 1 << 4,
        kLeftAnkleFault  = 1 << 5,
        kRightAnkleFault = 1 << 6,
        kCanBusOff       = 1 << 7,
        kImuFault        = 1 << 8,
    };

    enum class SysEvent : uint32_t
    {
        kNone            = 0 << 0,
        kEmergencyStop   = 1 << 0,
        kWakeup          = 1 << 1,
        kStartCalibrate  = 1 << 2,
        kStartAssist     = 1 << 3,
        kStopAssist      = 1 << 4,
        kEnterSleep      = 1 << 5,
        kClearFaults     = 1 << 6,
    };

    enum class LocoMode : uint8_t
    {
        kSitting = 0,
        kSitToStand,
        kStanding,
        kStandToSit,
        kWalking,
        kRampAscent,
        kRampDescent,
        kStairAscent,
        kStairDescent,
    };

    struct ArbiterOverride {
        LocoMode forced_locomode = LocoMode::kWalking;
        bool enable_locomode_override = false;
    };

    struct TelemetryConfig {
        bool enable = false;
        uint32_t pause_until_ms = 0;
    };

    ExoData() : left_side_(true), right_side_(false) {}
    ~ExoData() = default;

    /**< Two Side */
    SideData left_side_;
    SideData right_side_;
    AoData ao_data_;
    ImuData body_imu_;

    State state_ = State::kSleep;
    Error error_code_ = Error::kNone;
    SysEvent pending_events_ = SysEvent::kNone;
    LocoMode loco_mode_ = LocoMode::kWalking;
    ArbiterOverride override_usr_ = {.forced_locomode = LocoMode::kSitToStand, .enable_locomode_override = false};
    TelemetryConfig telemetry_config_ = {.enable = false, .pause_until_ms = 0};

    float user_weight_kg_ = 60.0f;
    float battery_voltage_ = 24.0f;
    bool do_test = false;
};

DEFINE_ENUM_CLASS_BITWISE_OPS(ExoData::Error)
DEFINE_ENUM_CLASS_BITWISE_OPS(ExoData::SysEvent)

struct ExoHardware
{
    FDCAN_HandleTypeDef &motor_can;           // 用于大疆/Robstride电机通信
    FDCAN_HandleTypeDef &dm_imu_can;          // 用于IMU数据通信
    SPI_HandleTypeDef &sensor_spi;           // 用于传感器 SPI 通信
    UART_HandleTypeDef &sensor_uart;          // 用于接收无线传感器数据
    UART_HandleTypeDef &shell_uart;           // 用于命令行 Shell / VOFA 调试
    UART_HandleTypeDef &left_mag_encoder_uart;    // 用于磁栅尺编码器1
    UART_HandleTypeDef &right_mag_encoder_uart;   // 用于磁栅尺编码器2
};

enum ExoJointCanID : uint8_t
{
    kLeftHip = 0x01,    // DM4340
    kRightHip = 0x02,   // DM4340
    kLeftKnee = 0x2B,   // RS01
    kRightKnee = 0x56,  // RS01
    kLeftAnkle = 0x2A,  // RS02
    kRightAnkle = 0x55, // RS02
};

class AnkleJoint
{
public:
    explicit AnkleJoint(bool is_left, ExoData &pe)  : pe_(pe), ps_(is_left ? pe_.left_side_ : pe_.right_side_), pj_(is_left ? pe_.left_side_.ankle_joint_ : pe_.right_side_.ankle_joint_), motor_(is_left ? ExoJointCanID::kLeftAnkle : ExoJointCanID::kRightAnkle) {}
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

    /** 踝关节控制参数 */
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
        kJointPosition,
        kOpenLoopTorque,
        kJointTorque,
    };

    explicit KneeJoint(bool is_left, ExoData &pe) : pe_(pe), ps_(is_left ? pe.left_side_ : pe.right_side_), pj_(is_left ? pe.left_side_.knee_joint_ : pe.right_side_.knee_joint_), motor_(is_left ? ExoJointCanID::kLeftKnee : ExoJointCanID::kRightKnee), joint_tor_pid_(2.5f, 0.0f, 0.0f, -1.0f, motor_.limit_current_) {}
    virtual ~KneeJoint() = default;
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
    PIDController joint_tor_pid_;
    KneeForceProfileGenerator force_profile_generator_;
    DisturbanceObserver disturbance_observer_{5.0f};

    void ImpedanceControl();
    void TorqueControl();
    void OpenLoopTorqueControl();
private:
    CtrlMode ctrl_mode_ = CtrlMode::kImpedance;
};

class KneeSeaJoint
{
public:
    enum class CtrlMode : uint8_t
    {
        kJointPosition,
        kSpringForce,
    };

    explicit KneeSeaJoint(bool is_left,  ExoData &pe, DjiEscHub &dji_esc_hub, UART_HandleTypeDef &huart) : pe_(pe), ps_(is_left ? pe.left_side_ : pe.right_side_), pj_(is_left ? pe.left_side_.knee_sea_joint_ : pe.right_side_.knee_sea_joint_), motor_(dji_esc_hub, is_left ? DjiEsc::EscId::kId1 : DjiEsc::EscId::kId2), mag_encoder_(huart), 
    joint_pos_pid_(5.0f, 1.0f, 0.0f, -100.0f, motor_.max_iqref_amp_),
    spring_force_pid_(1.0f, 0.1, 0.0, -100.0f, motor_.max_iqref_amp_) {}
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
    DjiEsc motor_;   /** 电调ID固定: 左膝id=1, 右膝id=2 */
    MagEncoder mag_encoder_;
    PIDController joint_pos_pid_; /** 必须放在motor_后面, 因为依赖其进行构造 */
    PIDController spring_force_pid_;  /** 必须放在motor_后面, 因为依赖其进行构造 */
    KneeForceProfileGenerator force_profile_generator_;

private:
    CtrlMode ctrl_mode_ = CtrlMode::kSpringForce;
};

class HipJoint
{
public:
    explicit HipJoint(bool is_left, ExoData &pe) : pe_(pe), ps_(is_left ? pe_.left_side_ : pe_.right_side_), pj_(is_left ? pe_.left_side_.hip_joint_ : pe_.right_side_.hip_joint_), motor_(is_left ? ExoJointCanID::kLeftHip : ExoJointCanID::kRightHip) {}
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

class FsrGaitEstimator
{
public:
    explicit FsrGaitEstimator(FsrGaitData &gait_data) : gait_data_(gait_data) {}
    virtual ~FsrGaitEstimator() = default;

    void Calibrate();
    void Update();
    void Reset();
private:
    void ProcessCalibration(FsrSensorData& sensor, bool& do_calibrate, bool& do_refinement);
    void ProcessSensorUpdate(FsrSensorData& sensor);

    float UpdateExpectedDuration();
    float UpdateExpectedStanceDuration();
    float UpdateExpectedSwingDuration();
    static inline bool SchmittTrigger(float value, bool is_last_high, float lower, float upper)
    {
        if (is_last_high)
        {
            return value < lower ? false : true;
        }
        else
        {
            return value > upper ? true : false;
        }
    }

    FsrGaitData &gait_data_;
};

class AdaptiveOscillator
{
public:
    explicit AdaptiveOscillator(ExoData &pe) : pe_(pe) {}
    virtual ~AdaptiveOscillator() = default;

    void Update();
    void Reset();
private:
    ExoData &pe_;

    static constexpr uint64_t kMaxTstrideUs = 3.0 * 1000000;
    static constexpr uint64_t kMinTstrideUs = 0.1 * 1000000;
    static constexpr uint64_t kMaxStoppingDurationUs = 0.5 * 1000000;
    static constexpr float kEmaTauS = 0.2f;
    static constexpr uint8_t kNumAOs = 3;

    uint64_t tprev_sys_us_ = 0;
    uint64_t left_tk_sys_us_ = 0;
    uint64_t right_tk_sys_us_ = 0;

    float v_phi_ = 10.0f;
    float v_omega_= 10.0f;
    float eta_ = 1.0f;
    float kp_ = 1.0f;
    float rho_ = -0.7f;

    float hat_x_ = 0.0f;
    float omega_ = _2PI * 1.0f;
    float phi_[kNumAOs] = {0.0f};
    float alpha_[kNumAOs] = {0.2f};
    float alpha0_ = 0.0f;
    float vel_energy_ema_ = 0.0f;

    float left_Pe_tilde_tk_ = 0.0f;
    float right_Pe_tilde_tk_ = 0.0f;
    float left_epsilon_phi_tk_ = 0.0f;
    float right_epsilon_phi_tk_ = 0.0f;
    float left_phi_e_ = 0.0f;
    float right_phi_e_ = 0.0f;
};

class Side
{
public:
    explicit Side(bool is_left, ExoData &pe, DjiEscHub &dji_esc_hub, UART_HandleTypeDef &huart) : pe_(pe), ps_(is_left ? pe_.left_side_ : pe_.right_side_), hip_joint_(is_left, pe), knee_joint_(is_left, pe), knee_sea_joint_(is_left, pe, dji_esc_hub, huart), ankle_joint_(is_left, pe), fsr_gait_estimator_(ps_.fsr_gait_data_) {}
    virtual ~Side() = default;
    void Calibrate();
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
};

class ExoShell : public Shell
{
public:
    explicit ExoShell(UART_HandleTypeDef &huart, Exo &exo);
    ~ExoShell() = default;

    void OnCmdSetLed(int argc, char **argv);
    void OnCmdSetLocoMode(int argc, char **argv);
private:
    Exo &exo_;
};

class BodyImu
{
public:
    explicit BodyImu(ExoData &pe) : pe_(pe), body_imu_(pe.body_imu_) {}
    virtual ~BodyImu() = default;

    void Read();

    ExoData &pe_;
    ImuData &body_imu_;
    Mahony mahony_filter_{1000.0f}; // HACK: 注意这里的采样频率要根据实际修改
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

    explicit DaMiaoImuHub(ExoData &pe, FDCAN_HandleTypeDef &hfdcan) : pe_(pe), hfdcan_(hfdcan) {}
    virtual ~DaMiaoImuHub() = default;

    void CanRxCallback(uint32_t can_id, const uint8_t *data);

    ExoData &pe_;
    FDCAN_HandleTypeDef &hfdcan_;
private:
    void UpdateImuData(ImuData &imu_data, const uint8_t *data);
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

class Exo
{
public:
    explicit Exo(ExoData &pe, ExoHardware &hw) : pe_(pe), hw_(hw), dji_esc_hub_(hw.motor_can), dm_imu_hub_(pe, hw.dm_imu_can), ao_(pe), body_imu_(pe), shell_(hw.shell_uart, *this), left_side_(true, pe, dji_esc_hub_, hw.left_mag_encoder_uart), right_side_(false, pe, dji_esc_hub_, hw.right_mag_encoder_uart) {}
    ~Exo() = default;

    void Initialize();
    void Run();
    void Read();
    void Calibrate();
    void ResetCalibrationFlags();
    void Estimate();
    void Standby();
    void Assist();
    void Shutdown();

    void CheckSystemHealth();
    void VofaSendTelemetry();
    bool IsMotorConnect();
    bool IsCalibrateDone();
    bool IsStopWalking();

    void CanRxCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t can_id, const uint8_t *data);
    void SensorUartReceiveDma(void);
    void ShellUartReceiveDma(void);
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
    AdaptiveOscillator ao_;
    BodyImu body_imu_;
    ExoShell shell_;
    StateLed state_led_;
    ChirpGenerator chirp_generator_{1.0f, 8.0f, 60.0f};
    Side left_side_;
    Side right_side_;

private:
    volatile bool spi_data_ready_ = false;
    volatile uint8_t spi_dma_readed_size_ = 0;
    volatile uint8_t spi_dma_reading_idx_ = 0;
    volatile uint8_t spi_dma_handling_idx_ = 1;

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

extern "C" {
void CallExoCanRxCallBack(Exo *exo, FDCAN_HandleTypeDef *hfdcan, uint32_t can_ext_id, const uint8_t *rx_data);
void CallExoUartRxCallback(Exo *exo, UART_HandleTypeDef *huart, uint16_t data_size);
void CallExoUartErrorCallback(Exo *exo, UART_HandleTypeDef *huart);
void CallExoSpiRxStart(Exo *exo);
void CallExoSpiRxCallback(Exo *exo);
void CallExoSpiErrorCallback(Exo *exo, SPI_HandleTypeDef *hspi);
}


#endif