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
#include "status_led.hpp"
#include "fsr.hpp"
#include "robstride.hpp"
#include "dm_motor.hpp"
#include "force_profile_generator.hpp"
#include "adaptive_oscillator.hpp"
#include "pid.hpp"
#include "disturbance_observer.hpp"
#include "shell.hpp"


/** Forward declarations */
class IMUData;
class JointData;
class SideData;
class ExoData;
class AnkleJoint;
class KneeJoint;
class Side;
class ExoShell;
class Exo;


#include <type_traits>

#define DEFINE_ENUM_CLASS_BITWISE_OPS(EnumType) \
    inline constexpr EnumType operator|(EnumType a, EnumType b) { \
        return static_cast<EnumType>(static_cast<std::underlying_type_t<EnumType>>(a) | static_cast<std::underlying_type_t<EnumType>>(b)); \
    } \
    inline constexpr EnumType& operator|=(EnumType& a, EnumType b) { \
        a = a | b; return a; \
    } \
    inline constexpr bool operator&(EnumType a, EnumType b) { \
        return (static_cast<std::underlying_type_t<EnumType>>(a) & static_cast<std::underlying_type_t<EnumType>>(b)) != 0; \
    } \
    inline constexpr EnumType operator~(EnumType a) { \
        return static_cast<EnumType>(~static_cast<std::underlying_type_t<EnumType>>(a)); \
    } \
    inline constexpr EnumType& operator&=(EnumType& a, EnumType b) { \
        a = static_cast<EnumType>(static_cast<std::underlying_type_t<EnumType>>(a) & static_cast<std::underlying_type_t<EnumType>>(b)); \
        return a; \
    }

/** The following definitions should align with the NRF54 code; NRF54 will send a foot_sensor_packet_t data by UART8 */
typedef struct foot_sensor_packet_t
{
    int32_t mV_heel;
    int32_t mV_toe;
    float mV_pull;
    float quatI;
    float quatJ;
    float quatK;
    float quatReal;
} foot_sensor_packet_t;

typedef struct exo_sensor_packet_t
{
    foot_sensor_packet_t left_foot;
    foot_sensor_packet_t right_foot;
} exo_sensor_packet_t;

class ImuData
{
public:
    ImuData(bool is_left_ = true);
    ~ImuData() = default;

    bool is_left_;
    bool is_used_;

    float quat_i_;
    float quat_j_;
    float quat_k_;
    float quat_real_;
};

class JointData
{
public:
    JointData(bool is_left = true);
    ~JointData() = default;

    bool is_left_;
    bool is_used_;

    float rom_rad_;
    float pos_rad_;
    float vel_radps_;
    float tor_Nm_;
};

class SideData
{
public:
    SideData(bool is_left = true);
    ~SideData() = default;

    bool is_left_;
    bool is_used_;

    JointData hip_joint_;
    JointData knee_joint_;
    JointData ankle_joint_;
    ImuData foot_imu_;
    float ankle_plantarflexion_force_N_;

    static const uint8_t kNumStepsAvg = 3;   
    uint32_t step_times_[kNumStepsAvg];
    uint32_t stance_times_[kNumStepsAvg];
    uint32_t swing_times_[kNumStepsAvg];

    uint32_t ground_strike_timestamp_;
    uint32_t prev_ground_strike_timestamp_;
    uint32_t toe_strike_timestamp_;
    uint32_t prev_toe_strike_timestamp_;
    uint32_t toe_off_timestamp_;
    uint32_t prev_toe_off_timestamp_;

    float percent_gait_;
    float percent_stance_;
    float percent_swing_;
    float expected_step_duration_;
    float expected_stance_duration_;
    float expected_swing_duration_;
    float expected_duration_window_upper_coeff_;
    float expected_duration_window_lower_coeff_;

    bool ground_strike_;
    bool toe_strike_;
    bool toe_off_;
    bool toe_on_;
    bool heel_contact_state_;
    bool toe_contact_state_;
    bool prev_heel_contact_state_;
    bool prev_toe_contact_state_;

    bool is_calibration_done_;
    bool do_calibration_toe_fsr_;
    bool do_calibration_refinement_toe_fsr_;
    bool do_calibration_heel_fsr_;
    bool do_calibration_refinement_heel_fsr_;
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
    /** 下面利用友元进行位运算符重载已换成使用DEFINE_ENUM_CLASS_BITWISE_OPS宏注册 */
    // friend Error operator|(Error a, Error b) {
    //     return static_cast<Error>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
    // }
    // friend Error& operator|=(Error& a, Error b) {
    //     a = a | b;
    //     return a;
    // }
    // friend bool operator&(Error a, Error b) {
    //     return (static_cast<uint32_t>(a) & static_cast<uint32_t>(b)) != 0;
    // }
    // friend Error operator~(Error a) {
    //     return static_cast<Error>(~static_cast<uint32_t>(a));
    // }
    // friend Error& operator&=(Error& a, Error b) {
    //     a = static_cast<Error>(static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
    //     return a;
    // }

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
        bool enable_locomode_override = false;
        LocoMode forced_locomode = LocoMode::kWalking;
    };

    ExoData();
    ~ExoData() = default;

    /**< Two Side */
    SideData left_side_;
    SideData right_side_;
    /**< Common */
    float user_weight_kg_;
    float battery_voltage_;
    bool enable_vofa_telemetry_ = false;
    uint32_t vofa_pause_until_ms_ = 0;

    State state_ = State::kSleep;
    Error error_code_ = Error::kNone;
    SysEvent pending_events_ = SysEvent::kNone;
    LocoMode loco_mode_ = LocoMode::kWalking;
    ArbiterOverride override_usr_;

    /**< For AO */
    float ao_left_phase_rad_;
    float ao_right_phase_rad_;
    uint32_t ao_left_event_cnt_;
    uint32_t ao_right_event_cnt_;
};

DEFINE_ENUM_CLASS_BITWISE_OPS(ExoData::Error)
DEFINE_ENUM_CLASS_BITWISE_OPS(ExoData::SysEvent)

class AnkleJoint
{
public:
    AnkleJoint(bool is_left, ExoData *exo_data);
    ~AnkleJoint() = default;
    void Calibrate();
    void Read();
    bool IsMotorConnect();
    void Shutdown();
    void Standby();
    void Assist();

    ExoData *pe_;
    SideData *ps_;
    JointData *pj_;
    AnkleForceProfileGenerator force_profile_generator_;
    Robstride motor_;
    PIDController pid_;
    float cable_pre_tensioned_position_;
    float cable_tensioned_position_;
    float cable_released_position_;
    float assistance_start_phase_percent_;
    float assistance_end_phase_percent_;
};


class KneeJoint
{
public:
    KneeJoint(bool is_left, ExoData *exo_data);
    ~KneeJoint() = default;
    void Calibrate();
    void Read();
    bool IsMotorConnect();
    void Shutdown();
    void Standby();
    void Assist();

    ExoData *pe_;
    SideData *ps_;
    JointData *pj_;
    KneeForceProfileGenerator force_profile_generator_;
    Robstride motor_;
    DisturbanceObserver disturbance_observer_;
    void ImpedanceControl();
};

class HipJoint
{
public:
    HipJoint(bool is_left, ExoData *exo_data);
    ~HipJoint() = default;
    void Calibrate();
    void Read();
    bool IsMotorConnect();
    void Shutdown();
    void Standby();
    void Assist();
    ExoData *pe_;
    SideData *ps_;
    JointData *pj_;
    // HipForceProfileGenerator force_profile_generator_;
    DMMotor motor_;
};

class Side
{
public:
    Side(bool is_left, ExoData *pe);
    ~Side() = default;
    void Calibrate();
    void Read();
    void Assist();
    bool IsMotorConnect();
    void Standby();
    void CalibrateFsr();
    void FsrTimeBaseGaitPhaseEstimate();
    void ClearStepTimeEstimate();
    void Shutdown();
    float UpdateExpectedDuration();
    float UpdateExpectedStanceDuration();
    float UpdateExpectedSwingDuration();
    ExoData *pe_;
    SideData *ps_;
    Fsr heel_fsr_;
    Fsr toe_fsr_;
    HipJoint hip_joint_;
    KneeJoint knee_joint_;
    AnkleJoint ankle_joint_;
};

class ExoShell : public Shell
{
public:
    ExoShell(Exo* ptr_exo);
    ~ExoShell() = default;

    void OnCmdSetLed(int argc, char** argv);
    void OnCmdVofaTelemetry(int argc, char** argv);
    void OnCmdSetLocoMode(int argc, char** argv);
private:
    Exo* ptr_exo_;
};


class Exo
{
public:
    Exo(ExoData *pe);
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

    void CanRxCallback(uint32_t can_id, uint8_t *data);
    void SensorUartRxCallback(uint8_t *data, uint16_t data_size);
    void UsrBLEUartRxCallback(uint8_t *data, uint16_t data_size);

    ExoData *pe_;
    StateLed state_led_;
    AdaptiveOscillator adaptive_oscilator_;
    Side left_side_;
    Side right_side_;
    ExoShell shell_;
private:
    static ExoData::SysEvent AllowedEventsForState(ExoData::State s);
    static inline void ClearNonCriticalEvents(ExoData* pe)
    {
        if (pe == nullptr) return;
        pe->pending_events_ &= ~ExoData::SysEvent::kWakeup;
        pe->pending_events_ &= ~ExoData::SysEvent::kStartCalibrate;
        pe->pending_events_ &= ~ExoData::SysEvent::kStartAssist;
        pe->pending_events_ &= ~ExoData::SysEvent::kStopAssist;
        pe->pending_events_ &= ~ExoData::SysEvent::kEnterSleep;
        pe->pending_events_ &= ~ExoData::SysEvent::kClearFaults;
    }
};

extern "C" {
void CallExoCanRxCallBack(Exo *ptr_exo, uint32_t can_ext_id, uint8_t *rx_data);
void CallExoSensorUartRxCallback(Exo *ptr_exo, uint8_t *data, uint16_t data_size);
void CallExoUsrBLEUartRxCallback(Exo *ptr_exo, uint8_t *data, uint16_t data_size);
}


#endif