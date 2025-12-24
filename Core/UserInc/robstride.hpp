/**
 * @file robstride.hpp
 * @author your name (you@domain.com)
 * @brief 固件版本 rs01-0.1.3.3 + 说明书版本 250227
 * @version 0.1
 * @date 2025-06-09
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef ROBSTRIDE_HPP
#define ROBSTRIDE_HPP

#include <cstdint>

/* 可读写单个参数的Idx列表, 用于通信类型17,18 */
enum RobstrideParamIdx
{
    run_mode        = 0x7005,   //运行模式, 0~5
    iq_ref          = 0x7006,   //电流模式Iq指令, -23.0~23.0 A
    spd_ref         = 0x700A,   //转速模式转速指令, -44.0~44.0 rad/s
    limit_torque    = 0x700B,   //转矩限制, 0~17Nm
    cur_kp          = 0x7010,   //电流的Kp, 默认值0.17
    cur_ki          = 0x7011,   //电流的Ki, 默认值0.012
    cur_filt_gain   = 0x7014,   //电流滤波系数, 0~1.0，默认值0.1
    loc_ref         = 0x7016,   //位置模式角度指令
    limit_spd       = 0x7017,   //CSP位置模式速度限制, 0~44rad/s
    limit_cur       = 0x7018,   //速度模式相电流限制, 0~23A
    mechPos         = 0x7019,   //负载端计圈机械角度
    iqf             = 0x701A,   //iq滤波值, -23~23A
    mechVel         = 0x701B,   //负载端转速, -44~44rad/s
    VBUS            = 0x701C,   //母线电压, V
    loc_kp          = 0x701E,   //位置的kp, 默认值40
    spd_kp          = 0x701F,   //速度的kp, 默认值6
    spd_ki          = 0x7020,   //速度的ki, 默认值0.02
    spd_filt_gain   = 0x7021,   //速度滤波系数, 默认值0.1
    acc_rad         = 0x7022,   //速度模式加速度, 20rad/s^2
    vel_max         = 0x7024,   //位置模式(PP)速度, 默认值10rad/s
    acc_set         = 0x7025,   //位置模式(PP)加速度, 默认值10rad/s^2
    EPScan_time     = 0x7026,   //上报时间设置, 1代表10ms，加1递增5ms, 默认值1
    canTimeout      = 0x7028,   //can超时阀值, 20000代表1s, 默认值0
    zero_sta        = 0x7029,   //零点标志位, 0代表0-2π,1代表-π-π
};

enum RobstrideRunMode
{
    kMotionMode    = 0x00,         //运控模式
    kPositionPPMode = 0x01,       //位置模式(PP)
    kSpeedMode = 0x02,             //速度模式
    kCurrentMode = 0x03,           //电流模式
    kGoZeroPosMode = 0x04,          //设零模式
    kPositionCSPMode = 0x05,      //位置模式(CSP)
};

typedef enum
{
    ROBSTRIDE_KEEP_FAULT = 0x00,       //保持故障状态
    ROBSTRIDE_CLEAR_FAULT,             //清除故障状态
}RobstrideFaultFlag;

enum RobstrideZeroFlag: uint8_t
{
    kZero_2PI = 0x00,
    kMinusPI_PI = 0x01,
};

enum RobstridePattern : uint8_t
{
    kPatternReset = 0,
    kPatternCali = 1,
    kPatternMotor = 2
};


class Robstride
{
public:
    Robstride(uint8_t can_id);
    ~Robstride () = default;

    uint8_t can_id_;             //CAN ID
    uint8_t mcu_id_;             //MCU唯一标识符[后8位，共64位]
    uint8_t run_mode_;           //电机运行模式
	uint8_t error_code_;         //错误代码, 通信类型2
    uint8_t fault_code_;         //故障代码, 通信类型21
    uint8_t pattern_;            //电机工作模式, 通信类型2 24
    uint16_t EPScan_time_;       //上报时间设置, 1代表10ms, 加1递增5ms, 默认1
    uint32_t can_timeout_;        //can超时阀值, 20000代表1s, 默认0
    RobstrideZeroFlag zero_sta_; //零点标志位，0代表0-2π,1代表-π-π, only for RS00
    /* 反馈 */
    float temperature_;          //当前温度, 单位摄氏度
    float vbus_;                 //母线电压, 单位V
    float position_;             //当前位置, 单位rad
    float speed_;                //当前速度, 单位rad/s
    float iq_;                   //当前电流, 单位A
    float iq_filt_;              //当前电流滤波值, 单位A
    float torque_;               //当前扭矩, 单位Nm
    /* 设定参数 */
    float position_ref_ ;        //参考位置, 单位rad, RS01/00:-4*pi~4*pi rad
    float speed_ref_;            //参考速度, 单位rad/s, RS01:-44~44rad/s, RS00:-33~33rad/s
    float iq_ref_;               //参考电流, 单位A, RS01:-23~23A, RS00:-16~16A
    float torque_forward_;           //参考扭矩, 单位Nm, RS01:-17~17Nm, RS00:-14~14Nm
    float acc_rad_;              //速度模式加速度, 单位rad/s^2, RS01/00:默认值20rad/s^2
    float vel_max_;              //位置模式(PP)速度, 单位rad/s, RS01/00:默认值10rad/s
    float acc_set_;              //位置模式(PP)加速度, 单位rad/s^2, RS01/00:默认值10rad/s^2
    /* 控制参数 */
    float position_kp_;          //位置环kp, RS01/00:默认值40
    float speed_kp_;             //速度环kp, RS01/00:默认值6
    float speed_ki_;             //速度环ki, RS01/00:默认值0.02
    float speed_filt_gain_;      //速度滤波系数, RS01/00:默认值0.1
    float current_kp_;           //电流环kp, RS01/00:默认值0.17
    float current_ki_;           //电流环ki, RS01/00:默认值0.012
    float current_filt_gain_;    //电流滤波系数, 0~1.0, RS01/00:默认值0.1
    float motion_mode_kp_;       //运控模式kp, RS01/00:0.0~500.0
    float motion_mode_kd_;       //运控模式kd, RS01/00:0.0~5.0
    float limit_torque_;         //转矩限制, RS01:0~17Nm, RS00:0~14Nm
    float limit_speed_;          //位置模式(CSP)速度限制, RS01:0~44rad/s, RS00:0~33rad/s
    float limit_current_;        //位置速度模式电流限制, RS01:0~23A, RS00:0~16A
    /* 其他 */
    uint8_t feedback_flag_;      //反馈标志位, 0:未接收, 1:已接收

    /* 说明书中的通信类型 */
    void ObtainDeviceIDRequest(void);
    void ObtainDeviceIDReceive(uint8_t *can_rxdata);
    void MotionControl(void);
    void StatusFeedbackRequest(void);
    void StatusFeedbackReceive(uint32_t can_ext_id, uint8_t *can_rxdata);
    void EnableMotor(void);
    void StopMotor(uint8_t do_clear_error);
    void SetMecposZero(void);
    void SetMotorCanID(uint8_t can_id);
    void ReadSingleParamRequest(uint16_t param_index);
    void ReadSingleParamReceive(uint32_t can_ext_id, uint8_t *can_rxdata);
    void SetSingleParam(uint16_t param_index, float value);
    void FaultFeedbackReceive(uint8_t *can_rxdata);
    void SetBaudRate(void);
    void StatusFeedbackAutoRequest(bool do_enable);
    void StatusFeedbackAutoReceive(uint32_t can_ext_id, uint8_t *can_rxdata);
    void SetProtocal(void);
    /* 封装 */
    void PositionControlPP(void);
    void PositionControlCSP(void);
    void SpeedControl(void);
    void CurrentControl(void);
    void GoZeroPosMode(void);
    void CanRxCallBack(uint32_t can_ext_id, uint8_t *can_rxdata);
    void SetRunMode(void);
};

#endif

