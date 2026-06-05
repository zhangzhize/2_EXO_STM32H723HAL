/**
 * @file    dji_esc.hpp
 * @author  Zhize Zhang (1601266836@qq.com)
 * @brief   大疆 RoboMaster 系列电调控制类(C610/C620)
 *
 * @details 支持 M2006(配C610电调) 和 M3508(配C620电调) 两种电机。
 *          通信采用 CAN 标准帧(11位ID):
 *          - 发送: ID=0x200 发送电调1~4的电流指令, ID=0x1FF 发送电调5~8的电流指令
 *            每个电调占 2 字节(大端序 int16), 数据区共 8 字节
 *          - 接收: ID=0x201~0x208 对应电调1~8的反馈帧
 *            每帧 8 字节: 位置(2B)|速度(2B)|电流(2B)|保留(2B)
 *
 *          控制架构采用集线器(Hub)模式:
 *          - DjiEscHub 管理同一 CAN 总线上的所有电调, 负责统一发送和接收分发
 *          - DjiEsc 负责单台电调的控制计算(PID级联)和反馈解析
 *          - 用户流程: 更新参考值 -> 调用控制函数 -> 调用 Hub.SendAllCanTxData() 统一发送
 * @version 0.1
 * @date    2026-04-09
 */
#ifndef DJI_ESC_HPP
#define DJI_ESC_HPP

#include <cstdint>
#include "pid.hpp"
#include "fdcan.h"

class DjiEsc;

/**
 * @class   DjiEscHub
 * @brief   大疆电调 CAN 总线集线器, 管理同一 CAN 总线上的 1~8 台电调
 *
 * @details 负责:
 *          - 注册电调节点 RegisterNode() — 将 DjiEsc 对象与电调 ID 绑定
 *          - 统一发送电流指令 SendAllCanTxData() — 将 8 台电调的 iq 参考值打包到 0x200 和 0x1FF 两帧中
 *          - 接收分发 CanRxCallBack() — 根据 CAN ID(0x201~0x208) 分发反馈数据到对应 DjiEsc 节点
 *
 *          设计用意: 大疆电调协议中 4 台电调共享一帧电流指令,
 *          由 Hub 统一发送可以避免多帧碎片化, 确保控制周期一致。
 */
class DjiEscHub
{
public:
    explicit DjiEscHub(FDCAN_HandleTypeDef &hfdcan) : hfdcan_(hfdcan) {}
    virtual ~DjiEscHub() = default;

    void SendAllCanTxData(); /*!< 将 8 台电调的 iq 参考值打包为 0x200 和 0x1FF 帧统一发送, 仅在对应 ESC 活跃时才发送 */
    void CanRxCallBack(uint32_t can_std_id, const uint8_t *rx_data); /*!< CAN 接收回调, 按 ID(0x201~0x208) 分发到对应节点 */

    void RegisterNode(uint8_t esc_id, DjiEsc *node); /*!< 注册电调节点, 将 DjiEsc 对象与电调 ID(1~8) 绑定 */
    inline void SetTxIqRef(uint8_t esc_id, int16_t iq_tx_data) /*!< 设置电调的电流发送值(由 DjiEsc 控制函数调用) */
    {
        if (esc_id >= 1 && esc_id <= 8) {
            tx_iq_refs_[esc_id - 1] = iq_tx_data;
            is_active_[esc_id - 1] = true; /* 标记该电调需要发送 */
        }
    }

private:
    FDCAN_HandleTypeDef &hfdcan_;    /*!< CAN 外设句柄引用 */
    int16_t tx_iq_refs_[8] = {0};    /*!< 8台电调的 Iq 参考值缓冲区, 索引=esc_id-1 */
    bool is_active_[8] = {false};    /*!< 活跃标志, 标记哪些电调有新数据需要发送 */

    DjiEsc *nodes_[8] = {nullptr};   /*!< 电调节点指针数组, 索引=esc_id-1, 用于接收分发 */
};

/**
 * @class   DjiEsc
 * @brief   大疆 RoboMaster 单台电调控制类
 *
 * @details 单台电调的反馈解析与控制计算, 支持三级联控制:
 *          - kPosition: 位置环(pos_pid_) -> 速度环(speed_pid_) -> 电流指令
 *          - kSpeed:    速度环(speed_pid_) -> 电流指令
 *          - kCurrent:  直接将 rotor_iq_reference_amp_ 限幅后发送
 *
 *          解码器为 8192 线增量编码器, 反馈位置为 0~8191 对应转子一周(0~2pi)。
 *          通过转子圈数累计和减速比换算得到输出轴绝对位置。
 *          电流指令用 int16 表示, 范围为 -max_iqref_to_can_data_ ~ +max_iqref_to_can_data_
 *          对应物理范围 -max_iqref_amp_ ~ +max_iqref_amp_ A。
 *
 * @note    大疆电调协议没有独立的"使能"/"失能"命令, 使能就是令 IqRef=0(但不关闭),
 *          失能就是令 IqRef=0 并设置 mode_=kStop。
 *          Rotor_round_count_ 依赖连续的位置反馈, 丢失反馈帧会导致圈数错乱。
 */
class DjiEsc
{
public:
    enum class EscId : uint8_t      /*!< 电调ID, 1~8, 对应反馈帧 0x201~0x208 */
    {
        kId1 = 1, kId2, kId3, kId4,
        kId5, kId6, kId7, kId8
    };
    enum class MotorType : uint8_t  /*!< 支持的电机类型, 决定转矩常数/最大电流/减速比等参数 */
    {
        kM2006, /*!< M2006 配 C610 电调: kt=0.18Nm/A, max_iq=10A, ratio=36:1 */
        kM3508  /*!< M3508 配 C620 电调: kt=0.30Nm/A, max_iq=20A, ratio≈19.2:1 */
    };
    enum class EscMode : uint8_t   /*!< 控制模式, 决定 PID 级联深度 */
    {
        kStop,      /*!< 停止模式, iq=0 */
        kPosition,  /*!< 位置控制: pos_pid -> speed_pid -> iq */
        kSpeed,     /*!< 速度控制: speed_pid -> iq */
        kCurrent    /*!< 电流控制: 直接发送 iq 参考值(需用户手动限幅) */
    };

    explicit DjiEsc(DjiEscHub &hub, EscId esc_id, MotorType motor_type = MotorType::kM3508); /*!< 构造函数, 需传入所属 Hub 和电调 ID, 自动注册到 Hub */
    virtual ~DjiEsc() = default;

    void UpdateFeedback(const uint8_t *rx_data);  /*!< 解析 CAN 反馈帧, 更新位置/速度/电流反馈, 由 Hub 自动调用 */
    void PositionControl();   /*!< 位置控制(级联PID), 调用前需设置 shaft_pos_reference_rad_ */
    void SpeedControl();      /*!< 速度控制(单级PID), 调用前需设置 shaft_speed_reference_radps_ */
    void CurrentControl();    /*!< 电流控制(直通), 调用前需设置 rotor_iq_reference_amp_ */
    void EnableMotor();       /*!< 使能: 令 iq=0 并复位 PID 误差(协议层面无使能命令) */
    void DisableMotor();      /*!< 失能: 令 iq=0, mode=kStop, 并复位 PID 误差 */

    PIDController pos_pid_;   /*!< 位置环PID控制器(输出端), 输出为速度参考, 需根据实际负载调试参数 */
    PIDController speed_pid_; /*!< 速度环PID控制器(输出端), 输出为 Iq 参考, 需根据实际负载调试参数 */

    float shaft_pos_feedback_rad_ = 0.0f;      /*!< 减速器输出端位置反馈(rad) */
    float shaft_pos_reference_rad_ = 0.0f;     /*!< 减速器输出端位置参考(rad), 写入后调用 PositionControl() */
    float shaft_speed_feedback_radps_ = 0.0f;  /*!< 减速器输出端速度反馈(rad/s) */
    float shaft_speed_reference_radps_ = 0.0f; /*!< 减速器输出端速度参考(rad/s), 写入后调用 SpeedControl() */
    float shaft_torque_feedback_Nm_ = 0.0f;    /*!< 减速器输出端转矩反馈(Nm), 由 iq * kt * ratio 估算 */
    float shaft_torque_reference_Nm_ = 0.0f;   /*!< 减速器输出端转矩参考(Nm), 暂未使用 */

    float rotor_iq_feedback_amp_ = 0.0f;        /*!< 转子端 Iq 电流反馈(A) */
    float rotor_iq_reference_amp_ = 0.0f;       /*!< 转子端 Iq 电流参考(A), 写入后调用 CurrentControl() */
    float rotor_pos_one_round_rad_ = 0.0f;      /*!< 转子在当前圈内的角度(0~2pi rad) */
    float rotor_last_pos_one_round_rad_ = 0.0f; /*!< 转子上次反馈的圈内角度, 用于过零检测和圈数累计 */
    int32_t rotor_round_count_ = 0;             /*!< 转子累计转动圈数(带符号), 用于计算绝对位置 */

    float kt_NmpA;                               /*!< 电机转矩常数(Nm/A), M2006=0.18, M3508=0.30 */
    float max_iqref_amp_;                       /*!< 最大 Iq 参考电流(A), M2006=10, M3508=20 */
    float reduction_ratio_;                     /*!< 减速比, M2006=36, M3508=3591/187(默认1, 减速箱已拆除) */
    int16_t max_iqref_to_can_data_;             /*!< 最大 Iq 参考电流对应的 CAN 数据值, M2006=10000, M3508=16384 */

    EscId esc_id_;                              /*!< 电调 ID(1~8) */
    MotorType motor_type_;                      /*!< 电机类型 */
    EscMode mode_ = EscMode::kStop;             /*!< 当前控制模式 */

private:
    static constexpr float kEncoderCountsPerRev = 8192.0f; /*!< 编码器每圈分辨率, 8192线=2pi rad */

    DjiEscHub& hub_; /*!< 所属 CAN 集线器引用, 用于 SetTxIqRef() 提交电流指令 */
};

#endif
