/**
 * @file dji_esc.hpp
 * @author Zhize Zhang (1601266836@qq.com)
 * @brief 大疆电调控制类, 支持M2006和M3508两种电机; 需要用户在回调函数里调用CanRxCallBack解析CAN数据, 调用PositionControl/SpeedControl/CurrentControl计算控制输出, 最后调用SendAllCanTxData统一发送CAN数据
 * @note 使用方法: 实例化DjiEsc类并调用相应的控制函数
 * @example 实例化对象: DjiEsc dj_esc(DjiEsc::MotorType::kM3508, DjiEsc::EscId::kId1, &hfdcan1); 
 * @example 位置控制: 设置位置参考值并调用位置控制函数, 如dj_esc.shaft_pos_reference_rad_ = 1.0f; dj_esc.PositionControl(); 所有ID的电调做了同样的操作后调用DjiEsc::SendAllCanTxData(&hfdcan1);
 * @version 0.1
 * @date 2026-04-09
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#ifndef DJI_ESC_HPP
#define DJI_ESC_HPP

#include <cstdint>
#include "pid.hpp"
#include "fdcan.h"

class DjiEsc
{
public:
    enum class EscId : uint8_t      /** 电调ID, 1-8 */
    {
        kId1 = 1, kId2, kId3, kId4, 
        kId5, kId6, kId7, kId8
    };    
    enum class MotorType : uint8_t  /** 支持的电机类型 */
    {
        kM2006,
        kM3508
    };
    enum class EscMode : uint8_t   /** 三种控制模式 */
    {
        kStop,
        kPosition,
        kSpeed,
        kCurrent
    };

    explicit DjiEsc(MotorType motor_type = MotorType::kM3508, EscId esc_id = EscId::kId1, FDCAN_HandleTypeDef *hfdcan = &hfdcan1);  /** 默认M3508, id=1, FDCAN1; 每个ID只能构造一个对象(这个要由用户保证) */
    virtual ~DjiEsc() = default;

    void CanRxCallBack(uint32_t can_std_id, uint8_t *rx_data);   /** 解析CAN数据 */
    void PositionControl();   /** 输出轴位置控制 */
    void SpeedControl();      /** 输出轴速度控制 */
    void CurrentControl();    /** q轴电流控制 */
    void EnableMotor();       /** 就是令iqref=0 */
    void DisableMotor();      /** 就是令iqref=0 */

    static void SendAllCanTxData(FDCAN_HandleTypeDef *hfdcan);     /** static全局函数, 当所有ID的对象使用SetTxIqRef配置好电流后再调用 */

    PIDController position_pid_;  /** pid参数需要调试 */
    PIDController speed_pid_;
    FDCAN_HandleTypeDef *hfdcan_; /** #HACK 需要根据实际配置 */

    float shaft_pos_feedback_rad_ = 0.0f;      /** 输出端位置反馈 */
    float shaft_pos_reference_rad_ = 0.0f;     /** 输出端位置参考, 写入后调用PositionControl() */
    float shaft_speed_feedback_radps_ = 0.0f;  /** 输出端速度反馈 */
    float shaft_speed_reference_radps_ = 0.0f; /** 输出端速度参考, 写入后调用SpeedControl() */

    float rotor_iq_feedback_amp_ = 0.0f;        /** q轴端电流反馈 */
    float rotor_iq_reference_amp_ = 0.0f;       /** q轴端电流参考, 写入后调用CurrentControl() */
    float rotor_pos_one_round_rad_ = 0.0f;      /** 转子一周的角度 */
    float rotor_last_pos_one_round_rad_ = 0.0f; /** 记录转子上次反馈的角度, 用于计算转子转动圈数 */
    int32_t rotor_round_count_ = 0;             /** 记录转子转动圈数 */

    float max_iqref_amp_;                       /** 最大电流参考值, M2006是10, M3508是20 */
    float reduction_ratio_;                     /** 减速比, M2006是36, M3508是3591/187 */
    int16_t max_iqref_to_can_data_;             /** 最大电流参考值, 对应的16位数据 */

    EscId esc_id_;                              /** 电调ID */
    MotorType motor_type_;                      /** 电调类型 */
    EscMode mode_ = EscMode::kStop;             /** 控制模式 */

private:
    void SetTxIqRef(int16_t iq_tx_data);

    static constexpr float kRxDataToPositionDeg = 8192.0f;
    static int16_t tx_iq_refs_[8];
    static bool is_id_active_[8];
};

#endif
