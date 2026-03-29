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
#include "exo_data.hpp"
#include "pid.hpp"
#include "disturbance_observer.hpp"

enum ExoJointCanID : uint8_t
{
    kLeftHip = 0x01,
    kRightHip = 0x02,
    kLeftKnee = 0x2A,
    kRightKnee = 0x55,
    kLeftAnkle = 0x2A,
    kRightAnkle = 0x55,
};


class AnkleJoint
{
public:
    AnkleJoint(bool is_left, ExoData *exo_data);
    ~AnkleJoint() = default;
    void Calibrate();
    void Read();
    void WaitForCommunication();
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
    float assistance_start_phase_rad_;
    float assistance_end_phase_rad_;
};


class KneeJoint
{
public:
    KneeJoint(bool is_left, ExoData *exo_data);
    ~KneeJoint() = default;
    void Calibrate();
    void Read();
    void WaitForCommunication();
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
    void WaitForCommunication();
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
    void CalibrateFsr();
    void EstimateGait();
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

class Exo
{
public:
    Exo(ExoData *pe);
    ~Exo() = default;
    void Initialize();
    void Run();
    void Calibrate();
    void Read();
    void Estimate();
    void Assist();
    void Shutdown();
    void ReadBatVol();
    void CanRxCallback(uint32_t can_id, uint8_t *data);
    void UartRxCallback(uint8_t *data, uint16_t data_size);
    ExoData *pe_;
    StatusLed status_led_;
    AdaptiveOscillator adaptive_oscilator_;
    Side left_side_;
    Side right_side_;
};


extern "C" {
void CallExoCanRxCallBack(Exo *ptr_exo, uint32_t can_ext_id, uint8_t *rx_data);
void CallExoUartRxCallBack(Exo *ptr_exo, uint8_t *data, uint16_t data_size);
}




#endif