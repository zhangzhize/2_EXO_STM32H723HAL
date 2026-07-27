/**
 * @file    robstride.hpp
 * @brief   灵足(Robstride)电机驱动器通信协议封装
 *
 * @details 
 * @version 0.1 (固件版本 rs01-0.1.3.3 + 说明书版本 250227)
 * @date    2025-06-09
 */
#ifndef ROBSTRIDE_HPP
#define ROBSTRIDE_HPP

#include <cstdint>
#include "fdcan.h"

enum RobstrideParamIdx
{
    run_mode        = 0x7005,   /*!< 运行模式, 0~5 对应各控制模式 */
    iq_ref          = 0x7006,   /*!< 电流模式 Iq 指令, -23.0~23.0 A */
    spd_ref         = 0x700A,   /*!< 转速模式转速指令, -44.0~44.0 rad/s */
    limit_torque    = 0x700B,   /*!< 转矩限制, 0~17Nm */
    cur_kp          = 0x7010,   /*!< 电流环 Kp, 默认值 0.17 */
    cur_ki          = 0x7011,   /*!< 电流环 Ki, 默认值 0.012 */
    cur_filt_gain   = 0x7014,   /*!< 电流滤波系数, 0~1.0, 默认值 0.1 */
    loc_ref         = 0x7016,   /*!< 位置模式角度指令(rad) */
    limit_spd       = 0x7017,   /*!< CSP位置模式速度限制, 0~44rad/s */
    limit_cur       = 0x7018,   /*!< 速度模式相电流限制, 0~23A */
    mechPos         = 0x7019,   /*!< 负载端计圈机械角度(rad), 读反馈用 */
    iqf             = 0x701A,   /*!< Iq 滤波值, -23~23A, 读反馈用 */
    mechVel         = 0x701B,   /*!< 负载端转速, -44~44rad/s, 读反馈用 */
    VBUS            = 0x701C,   /*!< 母线电压(V), 读反馈用 */
    loc_kp          = 0x701E,   /*!< 位置环 Kp, 默认值 40 */
    spd_kp          = 0x701F,   /*!< 速度环 Kp, 默认值 6 */
    spd_ki          = 0x7020,   /*!< 速度环 Ki, 默认值 0.02 */
    spd_filt_gain   = 0x7021,   /*!< 速度滤波系数, 默认值 0.1 */
    acc_rad         = 0x7022,   /*!< 速度模式加速度, 默认值 20rad/s^2 */
    vel_max         = 0x7024,   /*!< 位置模式(PP)速度, 默认值 10rad/s */
    acc_set         = 0x7025,   /*!< 位置模式(PP)加速度, 默认值 10rad/s^2 */
    EPScan_time     = 0x7026,   /*!< 上报时间间隔设置, 1代表10ms, 加1递增5ms, 默认值1 */
    canTimeout      = 0x7028,   /*!< CAN超时阈值, 20000代表1s, 默认值0(不启用) */
    zero_sta        = 0x7029,   /*!< 零点标志位, 0代表0~2π, 1代表-π~π, 仅RS00 */
};

enum RobstrideMotorMode
{
    kMotionMode    = 0x00,         /*!< 运控模式 */
    kPositionPPMode = 0x01,        /*!< 位置模式(PP) */
    kSpeedMode = 0x02,             /*!< 速度模式 */
    kCurrentMode = 0x03,           /*!< 电流模式 */
    kGoZeroPosMode = 0x04,         /*!< 设零模式 */
    kPositionCSPMode = 0x05,       /*!< 位置模式(CSP) */
};

/* 灵足电机故障处理标志, 用于 DisableMotor() 控制清零行为 */
typedef enum
{
    ROBSTRIDE_KEEP_FAULT = 0x00,       /*!< 保持故障状态, 不清除错误 */
    ROBSTRIDE_CLEAR_FAULT,             /*!< 清除故障状态 */
}RobstrideFaultFlag;

/* 灵足电机零点角度范围标志(仅 RS00), 对应参数 zero_sta(0x7029) */
enum RobstrideZeroFlag: uint8_t
{
    kZero_2PI = 0x00,       /*!< 零点范围: 0 ~ 2pi */
    kMinusPI_PI = 0x01,     /*!< 零点范围: -pi ~ +pi */
};

/** 灵足电机工作模式(Pattern), 通过反馈帧扩展ID的 bit22-23 获取 */
enum RobstridePattern : uint8_t
{
    kPatternReset = 0,      /*!< 复位模式 */
    kPatternCali = 1,       /*!< 标定模式 */
    kPatternMotor = 2       /*!< 电机正常运行模式 */
};


class Robstride
{
public:
    Robstride(FDCAN_HandleTypeDef &hfdcan, uint8_t can_id) :
        hfdcan_(hfdcan),
        can_id_(can_id)
    {
    }
    ~Robstride () = default;

    FDCAN_HandleTypeDef &hfdcan_;     /*!< 该电机固定使用的 FDCAN 外设 */
    uint8_t can_id_;                 /*!< 电机 CAN ID, 用于扩展帧低8位寻址和接收匹配 */
    uint8_t mcu_id_ = 0;             /*!< MCU唯一标识符[后8位，共64位], 由通信类型0获取 */
    uint8_t run_mode_ = kMotionMode; /*!< 电机当前运行模式, 切换前需先失能 */
	uint8_t error_code_ = 0;         /*!< 错误代码, 通过通信类型2反馈帧扩展ID bit16-21 解析 */
    uint8_t fault_code_ = 0;         /*!< 故障代码, 通过通信类型21故障帧获取 */
    uint8_t pattern_ = kPatternReset;/*!< 电机工作模式, 通过通信类型2反馈帧扩展ID bit22-23 解析 */
    uint16_t EPScan_time_ = 1;       /*!< 主动上报时间间隔, 1=10ms, 加1递增5ms */
    uint32_t can_timeout_ = 0;       /*!< CAN超时阈值, 20000=1s, 0=不启用 */
    RobstrideZeroFlag zero_sta_ = kMinusPI_PI; /*!< 零点标志位, 0=0~2pi, 1=-pi~+pi(仅RS00) */
    /* 反馈 */
    float temperature_ = 0.0f;       /*!< 当前温度(摄氏度) */
    float vbus_ = 0.0f;              /*!< 母线电压(V) */
    float position_ = 0.0f;          /*!< 当前负载端机械位置(rad) */
    float speed_ = 0.0f;             /*!< 当前负载端转速(rad/s) */
    float iq_ = 0.0f;                /*!< 当前 Iq 电流(A) */
    float iq_filt_ = 0.0f;           /*!< 当前 Iq 电流滤波值(A) */
    float torque_ = 0.0f;            /*!< 当前扭矩(Nm) */
    /* 设定参数 */
    float position_ref_ = 0.0f;       /*!< 参考位置(rad), RS01/00: -4pi~4pi */
    float speed_ref_ = 0.0f;          /*!< 参考速度(rad/s), RS01: -44~44, RS00: -33~33 */
    float iq_ref_ = 0.0f;             /*!< 参考电流(A), RS01: -23~23, RS00: -16~16 */
    float torque_forward_ = 0.0f;     /*!< 参考扭矩(Nm), 运控模式中作为前馈, RS01: -17~17, RS00: -14~14 */
    float acc_rad_ = 20.0f;           /*!< 速度模式加速度(rad/s^2), 默认值20 */
    float vel_max_ = 10.0f;           /*!< PP位置模式最大速度(rad/s), 默认值10 */
    float acc_set_ = 10.0f;           /*!< PP位置模式加速度(rad/s^2), 默认值10 */
    /* 控制参数 */
    float position_kp_ = 30.0f;       /*!< 位置环 Kp, RS01/00 默认值40 */
    float speed_kp_ = 1.0f;           /*!< 速度环 Kp, RS01/00 默认值6 */
    float speed_ki_ = 0.002f;         /*!< 速度环 Ki, RS01/00 默认值0.02 */
    float speed_filt_gain_ = 0.1f;    /*!< 速度滤波系数(0~1.0), RS01/00 默认值0.1 */
    float current_kp_ = 1.5f;         /*!< 电流环 Kp, RS01/00 默认值0.17 */
    float current_ki_ = 0.05f;        /*!< 电流环 Ki, RS01/00 默认值0.012 */
    float current_filt_gain_ = 0.01f; /*!< 电流滤波系数(0~1.0), RS01/00 默认值0.1 */
    float motion_mode_kp_ = 0.0f;     /*!< 运控模式 Kp(0.0~500.0) */
    float motion_mode_kd_ = 0.0f;     /*!< 运控模式 Kd(0.0~5.0) */
    float limit_torque_ = 17.0f;      /*!< 转矩限制(Nm), RS01: 0~17, RS00: 0~14 */
    float limit_speed_ = 44.0f;       /*!< CSP位置模式速度限制(rad/s), RS01: 0~44, RS00: 0~33 */
    float limit_current_ = 23.0f;     /*!< 速度/位置模式电流限制(A), RS01: 0~23, RS00: 0~16 */
    /* 其他 */
    uint8_t status_feedback_cnt_ = 0; /*!< 反馈计数器, 用于监控通信类型2帧的接收频率 */

    /* ——————————————— 通信类型方法 ——————————————— */

    void ObtainDeviceIDRequest(void);                                /*!< 通信类型0:  获取设备ID和MCU唯一标识符(发送请求) */
    void ObtainDeviceIDReceive(const uint8_t *can_rxdata);          /*!< 通信类型0:  获取设备ID的应答帧解析 */
    void MotionControl(void);                                       /*!< 通信类型1:  运控模式控制指令, pos+vel+kp+kd+torq 联合编码发送 */
    void StatusFeedbackRequest(void);                               /*!< 通信类型22: 电机数据保存帧请求 */
    void StatusFeedbackReceive(uint32_t can_ext_id, const uint8_t *can_rxdata); /*!< 通信类型2:  电机反馈数据解析(应答帧) */
    void EnableMotor(void);                                         /*!< 通信类型3:  电机使能运行 */
    void DisableMotor(uint8_t do_clear_error);                      /*!< 通信类型4:  电机停止运行, do_clear_error 控制是否清除错误 */
    void SetMecPosZero(void);                                       /*!< 通信类型6:  将当前位置设为机械零点(掉电丢失) */
    void SetMotorCanID(uint8_t can_id);                             /*!< 通信类型7:  设置电机CAN_ID(立即生效) */
    void ReadSingleParamRequest(uint16_t param_index);              /*!< 通信类型17: 读取单个参数(发送请求) */
    void ReadSingleParamReceive(uint32_t can_ext_id, const uint8_t *can_rxdata); /*!< 通信类型17: 读取单个参数的应答帧解析 */
    void SetSingleParam(uint16_t param_index, float value);         /*!< 通信类型18: 写入单个参数(掉电丢失), 应答帧为通信类型2 */
    void FaultFeedbackReceive(const uint8_t *can_rxdata);           /*!< 通信类型21: 故障反馈帧解析(电机会自动发送) */
    void SetBaudRate(void);                                         /*!< 通信类型23: CAN波特率修改(重新上电生效, 暂未实现) */
    void StatusFeedbackAutoRequest(bool do_enable);                 /*!< 通信类型24: 电机主动上报帧控制(false关闭/true开启) */
    void StatusFeedbackAutoReceive(uint32_t can_ext_id, const uint8_t *can_rxdata); /*!< 通信类型24: 主动上报帧的应答帧解析 */
    void SetProtocal(void);                                         /*!< 通信类型25: 电机协议修改(重新上电生效, 暂未实现) */
    /* ——————————————— 高级封装方法 ——————————————— */
    void PositionControlPP(void);   /*!< PP位置模式控制, 自动切换模式并设置目标位置+速度和加速度限制 */
    void PositionControlCSP(void);  /*!< CSP位置模式控制, 自动切换模式并设置目标位置+速度限制(适合实时轨迹跟踪) */
    void SpeedControl(void);        /*!< 速度模式控制, 自动切换模式并设置目标速度+电流和加速度限制 */
    void CurrentControl(void);      /*!< 电流模式控制, 自动切换模式并设置 Iq 参考值 */
    void GoZeroPosMode(void);       /*!< 将当前位置设为机械零点(通过切换模式实现) */
    void CanRxCallBack(uint32_t can_ext_id, const uint8_t *can_rxdata); /*!< CAN接收回调, 根据通信类型分发到对应解析函数 */
    void SetMotorMode(void);        /*!< 通过 SetSingleParam 将 run_mode_ 同步到电机(需先失能电机) */

private:
    void SendData(uint32_t can_ext_id, uint8_t *data, uint32_t data_size);
};

#endif
