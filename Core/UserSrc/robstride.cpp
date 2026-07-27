#include "robstride.hpp"

#include "bsp_can.h"

/* 灵足电机 CAN 扩展帧通信类型编码(占扩展ID bit24~28) */
enum ComTypeCode
{
    kComTypeObtainDeviceID = 0x00,      /*!< 获取设备ID和64位MCU唯一标识符 */
    kComTypeMotionControl = 0x01,       /*!< 发送运控模式电机控制指令(pos+vel+kp+kd) */
    kComTypeStatusFeedback = 0x02,      /*!< 电机反馈数据(pos+vel+torq+温度) */
    kComTypeEnableMotor = 0x03,         /*!< 电机使能运行 */
    kComTypeDisableMotor = 0x04,        /*!< 电机停止运行 */
    kComTypeSetMechPosZero = 0x06,      /*!< 把当前电机位置设为机械零位(掉电丢失) */
    kComTypeSetMotorCanID = 0x07,       /*!< 设置电机CAN_ID, 立即生效, 需重启后生效 */
    kComTypeReadSingleParam = 0x11,     /*!< 单个参数读取(如运行模式, PID增益等) */
    kComTypeSetSingleParam = 0x12,      /*!< 单个参数写入(掉电丢失), 用于非运控模式设置目标值 */
    kComTypeFaultFeedback = 0x15,       /*!< 故障反馈帧(电机自动上报) */
    kComTypeStatusFeedbackReq = 0x16,   /*!< 电机数据保存帧, 请求电机反馈状态 */
    kComTypeSetBaudRate = 0x17,         /*!< 电机波特率修改帧(重新上电生效) */
    kComTypeStatusFeedbackAuto = 0x18,  /*!< 电机主动上报帧(定时自动发送状态数据) */
};

#define Master_CAN_ID (0x00)  /*!< 主机 CAN ID, 固定为 0x00 */
/* 控制参数范围最值, 谨慎更改 — 用于运控模式下定点数归一化映射 */
#define P_MIN (-12.57f)  /*!< 位置最小值(rad), 即 -4pi */
#define P_MAX (12.57f)   /*!< 位置最大值(rad), 即 +4pi */
#define V_MIN (-44.0f)   /*!< 速度最小值(rad/s) */
#define V_MAX (44.0f)    /*!< 速度最大值(rad/s) */
#define T_MIN (-17.0f)   /*!< 扭矩最小值(Nm) */
#define T_MAX (17.0f)    /*!< 扭矩最大值(Nm) */

#define KP_MIN (0.0f)    /*!< 运控模式 Kp 最小值 */
#define KP_MAX (500.0f)  /*!< 运控模式 Kp 最大值 */
#define KD_MIN (0.0f)    /*!< 运控模式 Kd 最小值 */
#define KD_MAX (5.0f)    /*!< 运控模式 Kd 最大值 */

void Robstride::SendData(uint32_t can_ext_id, uint8_t *data, uint32_t data_size)
{
    FDCanSendData(&hfdcan_, can_ext_id, FDCAN_EXTENDED_ID, data, data_size);
}

/**
 * @brief          16位回文数据转浮点
 * @param[in]      x:16位回文
 * @param[in]      x_min:对应参数下限
 * @param[in]      x_max:对应参数上限
 * @param[in]      num_bits:参数位数
 * @retval         返回浮点值
 */
static float UintToFloat(uint32_t x, float x_min, float x_max, int num_bits)
{
    uint32_t span = (1 << num_bits) - 1;
    float offset = x_max - x_min;
    return offset * x / span + x_min;
}
/**
 * @brief          浮点转16位发文数据
 * @param[in]      x:浮点
 * @param[in]      x_min:对应参数下限
 * @param[in]      x_max:对应参数上限
 * @param[in]      num_bits:参数位数
 * @retval         返回16位发文数据
 */
static int FloatToUint(float x, float x_min, float x_max, int num_bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max)
        x = x_max;
    else if (x < x_min)
        x = x_min;
    return (int)((x - offset) * ((float)((1 << num_bits) - 1)) / span);
}

/**
 * @brief          通信类型0:  获取设备的ID和64位MCU唯一标识符(发送请求)
 * @param[in]      无
 * @retval         无
 */
void Robstride::ObtainDeviceIDRequest(void)
{
    uint32_t can_ext_id = 0;
    uint8_t can_txdata[8] = {0};

    can_ext_id = kComTypeObtainDeviceID << 24 | Master_CAN_ID << 8 | can_id_;
    SendData(can_ext_id, can_txdata, sizeof(can_txdata));
}

/**
 * @brief          通信类型0:  获取设备的ID和64位MCU唯一标识符(应答帧)
 * @param[in]      can_rxdata:CAN接收数据帧的数据区, 8字节
 * @retval         无
 */
void Robstride::ObtainDeviceIDReceive(const uint8_t *can_rxdata) { mcu_id_ = can_rxdata[0]; }

/**
 * @brief          通信类型1:  发送运控模式电机控制指令; 应答帧为通信类型2
 * @param[in]      无
 * @retval         无
 */
void Robstride::MotionControl(void)
{
    if (run_mode_ != RobstrideMotorMode::kMotionMode)
    {
        run_mode_ = RobstrideMotorMode::kMotionMode;
        SetMotorMode();
        EnableMotor();
    }
    if (pattern_ != RobstridePattern::kPatternMotor)
    {
        EnableMotor();
    }

    int16_t temp = 0;
    uint32_t can_ext_id = 0;
    uint8_t can_txdata[8] = {0};

    temp = FloatToUint(position_ref_, P_MIN, P_MAX, 16); /* 目标角度[0~65536] (-4pi~4pi)rad */
    can_txdata[0] = temp >> 8;
    can_txdata[1] = temp;
    temp = FloatToUint(speed_ref_, V_MIN, V_MAX, 16); /* 目标角速度[0~65536] (-44~44)rad/s */
    can_txdata[2] = temp >> 8;
    can_txdata[3] = temp;
    temp = FloatToUint(motion_mode_kp_, KP_MIN, KP_MAX, 16); /* [0~65536] (0.0~500.0) */
    can_txdata[4] = temp >> 8;
    can_txdata[5] = temp;
    temp = FloatToUint(motion_mode_kd_, KD_MIN, KD_MAX, 16); /* [0~65536] (0.0~5.0) */
    can_txdata[6] = temp >> 8;
    can_txdata[7] = temp;

    /* 扩展ID编码: bit24-28=通信类型1, bit8-23=扭矩前馈(16bit定点), bit0-7=电机CAN_ID */
    can_ext_id = kComTypeMotionControl << 24 | FloatToUint(torque_forward_, T_MIN, T_MAX, 16) << 8 | can_id_;
    SendData(can_ext_id, can_txdata, sizeof(can_txdata));
}

/**
 * @brief          通信类型2:  电机反馈数据(应答帧)
 * @param[in]      robstride_motor:灵足电机结构体指针
 * @retval         无
 */
void Robstride::StatusFeedbackReceive(uint32_t can_ext_id, const uint8_t *can_rxdata)
{
    /* 反馈数据区: 位置(16bit) | 速度(16bit) | 扭矩(16bit) | 温度(16bit), 均为定点数 */
    position_ = UintToFloat(can_rxdata[0] << 8 | can_rxdata[1], P_MIN, P_MAX, 16); /* 当前角度[0~65536] (-4pi~4pi)rad */
    speed_ = UintToFloat(can_rxdata[2] << 8 | can_rxdata[3], V_MIN, V_MAX, 16);  /* 当前角速度[0~65536] (-44~44)rad/s */
    torque_ = UintToFloat(can_rxdata[4] << 8 | can_rxdata[5], T_MIN, T_MAX, 16); /* 当前力矩[0~65536] (-17~17)Nm */
    temperature_ = (can_rxdata[6] << 8 | can_rxdata[7]) * 0.1; /* 温度 = 定点值 * 0.1 (摄氏度) */
    /* 扩展ID bit16-21 携带错误代码(6bit), 非0表示电机故障 */
    error_code_ = (can_ext_id & 0x3F0000) >> 16;
    if (error_code_ != 0x00)
    {
        DisableMotor(ROBSTRIDE_KEEP_FAULT); /* 故障时保持状态不清除, 等待外部处理 */
    }
    /* 扩展ID bit22-23 携带电机工作模式(2bit), 0=复位 1=标定 2=正常 */
    pattern_ = (can_ext_id & 0xC00000) >> 22;
}

/**
 * @brief          通信类型3:  电机使能运行; 应答帧为通信类型2
 * @param[in]      robstride_motor:灵足电机结构体指针
 * @retval         无
 */
void Robstride::EnableMotor(void)
{
    uint32_t can_ext_id = 0;
    uint8_t can_txdata[8] = {0};

    can_ext_id = kComTypeEnableMotor << 24 | Master_CAN_ID << 8 | can_id_;
    SendData(can_ext_id, can_txdata, sizeof(can_txdata));
}

/**
 * @brief          通信类型4:  电机停止运行; 应答帧为通信类型2
 * @param[in]      robstride_motor:灵足电机结构体指针
 * @param[in]      do_clear_error:清除错误位(0不清除 1清除)
 * @retval         无
 */
void Robstride::DisableMotor(uint8_t do_clear_error)
{
    uint32_t can_ext_id = 0;
    uint8_t can_txdata[8] = {0};

    can_txdata[0] = do_clear_error;  // 清除错误位设置
    can_ext_id = kComTypeDisableMotor << 24 | Master_CAN_ID << 8 | can_id_;
    SendData(can_ext_id, can_txdata, sizeof(can_txdata));
}

/**
 * @brief          通信类型6:  把当前电机位置设为机械零位(掉电丢失???); 应答帧为通信类型2
 * @param[in]      robstride_motor:灵足电机结构体指针
 * @retval         无
 */
void Robstride::SetMecPosZero(void)
{
    DisableMotor(0);
    uint32_t can_ext_id = 0;
    uint8_t can_txdata[8] = {0};

    can_txdata[0] = 1;
    can_ext_id = kComTypeSetMechPosZero << 24 | Master_CAN_ID << 8 | can_id_;
    SendData(can_ext_id, can_txdata, sizeof(can_txdata));
    EnableMotor();
}

/**
 * @brief          通信类型7:  设置电机CAN_ID, 立即生效; 应答帧为通信类型0
 * @param[in]      robstride_motor:灵足电机结构体数组
 * @param[in]      can_id:想要设置的CAN_ID
 * @retval         无
 */
void Robstride::SetMotorCanID(uint8_t can_id)
{
    DisableMotor(0);
    uint32_t can_ext_id = 0;
    uint8_t can_txdata[8] = {0};
    if (can_id_ != can_id)
    {
        can_ext_id = kComTypeSetMotorCanID << 24 | can_id << 16 | Master_CAN_ID << 8 | can_id;
        can_id_ = can_id;
        SendData(can_ext_id, can_txdata, sizeof(can_txdata));
    }
}

/**
 * @brief          通信类型17: 单个参数读取(发送请求)
 * @param[in]      robstride_motor:灵足电机结构体数组
 * @param[in]      param_index:参数Idx
 * @retval         无
 */
void Robstride::ReadSingleParamRequest(uint16_t param_index)
{
    uint32_t can_ext_id = 0;
    uint8_t can_txdata[8] = {0};
    can_ext_id = kComTypeReadSingleParam << 24 | Master_CAN_ID << 8 | can_id_;
    can_txdata[0] = param_index;       // 低8位
    can_txdata[1] = param_index >> 8;  // 高8位
    SendData(can_ext_id, can_txdata, sizeof(can_txdata));
}

/**
 * @brief          通信类型17: 单个参数读取(应答帧)
 * @param[in]      robstride_motor:灵足电机结构体数组
 * @param[in]      can_rxdata:CAN接收数据帧的数据区, 8字节
 * @retval         无
 */
void Robstride::ReadSingleParamReceive(uint32_t can_ext_id, const uint8_t *can_rxdata)
{
    uint16_t param_index = 0;
    uint8_t bytes[4] = {can_rxdata[4], can_rxdata[5], can_rxdata[6], can_rxdata[7]};
    if ((can_ext_id & 0xFF0000) >> 16 != 0x00)
    {
        return;
    }
    if (can_rxdata[1] != 0x70)
    {
        return;
    }

    float *ptr_float_data = (float *)bytes;
    param_index = can_rxdata[0] | can_rxdata[1] << 8;
    switch (param_index)
    {
        case RobstrideParamIdx::run_mode:
            run_mode_ = can_rxdata[4];
            break;
        case RobstrideParamIdx::iq_ref:
            iq_ref_ = *ptr_float_data;
            break;
        case RobstrideParamIdx::spd_ref:
            speed_ref_ = *ptr_float_data;
            break;
        case RobstrideParamIdx::limit_torque:
            limit_torque_ = *ptr_float_data;
            break;
        case RobstrideParamIdx::cur_kp:
            current_kp_ = *ptr_float_data;
            break;
        case RobstrideParamIdx::cur_ki:
            current_ki_ = *ptr_float_data;
            break;
        case RobstrideParamIdx::cur_filt_gain:
            current_filt_gain_ = *ptr_float_data;
            break;
        case RobstrideParamIdx::loc_ref:
            position_ref_ = *ptr_float_data;
            break;
        case RobstrideParamIdx::limit_spd:
            limit_speed_ = *ptr_float_data;
            break;
        case RobstrideParamIdx::limit_cur:
            limit_current_ = *ptr_float_data;
            break;
        case RobstrideParamIdx::mechPos:
            position_ = *ptr_float_data;
            break;
        case RobstrideParamIdx::iqf:
            iq_filt_ = *ptr_float_data;
            break;
        case RobstrideParamIdx::mechVel:
            speed_ = *ptr_float_data;
            break;
        case RobstrideParamIdx::VBUS:
            vbus_ = *ptr_float_data;
            break;
        case RobstrideParamIdx::loc_kp:
            position_kp_ = *ptr_float_data;
            break;
        case RobstrideParamIdx::spd_kp:
            speed_kp_ = *ptr_float_data;
            break;
        case RobstrideParamIdx::spd_ki:
            speed_ki_ = *ptr_float_data;
            break;
        case RobstrideParamIdx::spd_filt_gain:
            speed_filt_gain_ = *ptr_float_data;
            break;
        case RobstrideParamIdx::acc_rad:
            acc_rad_ = *ptr_float_data;
            break;
        case RobstrideParamIdx::vel_max:
            vel_max_ = *ptr_float_data;
            break;
        case RobstrideParamIdx::acc_set:
            acc_set_ = *ptr_float_data;
            break;
        case RobstrideParamIdx::EPScan_time:
            EPScan_time_ = can_rxdata[4] | can_rxdata[5] << 8;
            break;
        case RobstrideParamIdx::canTimeout:
            can_timeout_ = can_rxdata[4] | can_rxdata[5] << 8 | can_rxdata[6] << 16 | can_rxdata[7] << 24;
            break;
        default:
            break;
    }
}

/**
 * @brief          通信类型18: 单个参数写入(掉电丢失); 应答帧为通信类型2
 * @param[in]      robstride_motor:灵足电机结构体数组
 * @param[in]      param_index:参数Idx
 * @param[in]      Value:写入的参数值
 * @retval         无
 */
void Robstride::SetSingleParam(uint16_t param_index, float value)
{
    uint32_t can_ext_id = 0;
    uint8_t can_txdata[8] = {0};
    uint8_t bytes[4] = {0};
    uint16_t temp_u16 = 0;
    uint32_t temp_u32 = 0;

    can_ext_id = kComTypeSetSingleParam << 24 | Master_CAN_ID << 8 | can_id_;

    can_txdata[0] = param_index;
    can_txdata[1] = param_index >> 8;
    can_txdata[2] = 0x00;
    can_txdata[3] = 0x00;

    if (param_index == RobstrideParamIdx::run_mode)
    {
        can_txdata[4] = (uint8_t)value;
        can_txdata[5] = 0x00;
        can_txdata[6] = 0x00;
        can_txdata[7] = 0x00;
    }
    else if (param_index == RobstrideParamIdx::EPScan_time)
    {
        temp_u16 = (uint16_t)value;
        can_txdata[4] = temp_u16 & 0x00FF;
        can_txdata[5] = (temp_u16 & 0xFF00) >> 8;
        can_txdata[6] = 0x00;
        can_txdata[7] = 0x00;
    }
    else if (param_index == RobstrideParamIdx::canTimeout)
    {
        temp_u32 = (uint32_t)value;
        can_txdata[4] = temp_u32 & 0x000000FF;
        can_txdata[5] = (temp_u32 & 0x0000FF00) >> 8;
        can_txdata[6] = (temp_u32 & 0x0000FF00) >> 16;
        can_txdata[7] = (temp_u32 & 0x0000FF00) >> 24;
    }
    else
    {
        *(float *)bytes = value;
        can_txdata[4] = bytes[0];
        can_txdata[5] = bytes[1];
        can_txdata[6] = bytes[2];
        can_txdata[7] = bytes[3];
    }
    SendData(can_ext_id, can_txdata, sizeof(can_txdata));
}

/**
 * @brief          通信类型21:  电机故障反馈帧
 * @param[in]      robstride_motor:灵足电机结构体数组
 * @param[in]      can_rxdata:CAN接收数据帧的数据区, 8字节
 * @retval         无
 */
void Robstride::FaultFeedbackReceive(const uint8_t *can_rxdata)
{
    fault_code_ = can_rxdata[0] << 24 | can_rxdata[1] << 16 | can_rxdata[2] << 8 | can_rxdata[3];
    DisableMotor(ROBSTRIDE_KEEP_FAULT);
}

/**
 * @brief          通信类型22: 电机数据保存帧, 请求; 应答帧为通信类型2
 * @param[in]      robstride_motor:灵足电机结构体指针
 * @retval         无
 */
void Robstride::StatusFeedbackRequest(void)
{
    uint32_t can_ext_id = 0;
    uint8_t can_txdata[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};

    can_ext_id = kComTypeStatusFeedbackReq << 24 | Master_CAN_ID << 8 | can_id_;
    SendData(can_ext_id, can_txdata, sizeof(can_txdata));
}

/**
 * @brief          通信类型23: CAN通信波特率
 * @param[in]      robstride_motor:灵足电机结构体数组
 * @retval         无
 */
void Robstride::SetBaudRate(void) { /** 暂时不打算实现 */ }

/**
 * @brief          通信类型24: 电机主动上报帧
 * @param[in]      robstride_motor:灵足电机结构体数组
 * @param[in]      do_enable: false关闭 true开启
 * @retval         无
 */
void Robstride::StatusFeedbackAutoRequest(bool do_enable)
{
    uint32_t can_ext_id = 0;
    uint8_t can_txdata[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x00, 0x00};

    SetSingleParam(RobstrideParamIdx::EPScan_time, EPScan_time_);
    can_ext_id = kComTypeStatusFeedbackAuto << 24 | Master_CAN_ID << 8 | can_id_;
    can_txdata[6] = do_enable ? 0x01 : 0x00;
    SendData(can_ext_id, can_txdata, sizeof(can_txdata));
}

/**
 * @brief          通信类型24: 电机主动上报(应答帧)
 * @param[in]      robstride_motor:灵足电机结构体数组
 * @retval         无
 */
void Robstride::StatusFeedbackAutoReceive(uint32_t can_ext_id, const uint8_t *can_rxdata)
{
    position_ = UintToFloat(can_rxdata[0] << 8 | can_rxdata[1], P_MIN, P_MAX, 16); /* 当前角度[0~65536] (-4pi~4pi)rad */
    speed_ = UintToFloat(can_rxdata[2] << 8 | can_rxdata[3], V_MIN, V_MAX, 16);  /* 当前角速度[0~65536] (-44~44)rad/s */
    torque_ = UintToFloat(can_rxdata[4] << 8 | can_rxdata[5], T_MIN, T_MAX, 16); /* 当前力矩[0~65536] (-17~17)Nm */
    temperature_ = (can_rxdata[6] << 8 | can_rxdata[7]) * 0.1;
    error_code_ = (can_ext_id & 0x3F0000) >> 16;
    pattern_ = (can_ext_id & 0xC00000) >> 22;

    if (error_code_ != 0x00)
    {
        DisableMotor(ROBSTRIDE_KEEP_FAULT);
    }
}

/**
 * @brief          通信类型25: 电机协议修改帧, 重新上电生效
 * @param[in]      robstride_motor:灵足电机结构体数组
 * @retval         无
 */
void Robstride::SetProtocal(void) { /** 暂时不打算实现 */ }

/**
 * @brief          设置电机运行模式(必须停止时调整！)
 * @param[in]      robstride_motor:灵足电机结构体数组
 * @retval         无
 */
void Robstride::SetMotorMode(void)
{
    DisableMotor(ROBSTRIDE_KEEP_FAULT);
    SetSingleParam(RobstrideParamIdx::run_mode, run_mode_);
}

/**
 * @brief          发送PP位置控制模式电机控制指令
 * @param[in]      robstride_motor:灵足电机结构体指针
 * @retval         无
 */
void Robstride::PositionControlPP(void)
{
    if (run_mode_ != RobstrideMotorMode::kPositionPPMode)
    {
        run_mode_ = RobstrideMotorMode::kPositionPPMode;
        SetMotorMode();
        EnableMotor();
    }
    SetSingleParam(RobstrideParamIdx::vel_max, vel_max_);
    SetSingleParam(RobstrideParamIdx::acc_set, acc_set_);
    SetSingleParam(RobstrideParamIdx::loc_ref, position_ref_);
}

/**
 * @brief          发送CSP位置控制模式电机控制指令
 * @param[in]      robstride_motor:灵足电机结构体指针
 * @retval         无
 */
void Robstride::PositionControlCSP(void)
{
    if (run_mode_ != RobstrideMotorMode::kPositionCSPMode)
    {
        run_mode_ = RobstrideMotorMode::kPositionCSPMode;
        SetMotorMode();
        EnableMotor();
    }
    SetSingleParam(RobstrideParamIdx::limit_spd, limit_speed_);
    SetSingleParam(RobstrideParamIdx::loc_ref, position_ref_);
}

/**
 * @brief          发送速度控制模式电机控制指令
 * @param[in]      robstride_motor:灵足电机结构体指针
 * @retval         无
 */
void Robstride::SpeedControl(void)
{
    if (run_mode_ != RobstrideMotorMode::kSpeedMode)
    {
        run_mode_ = RobstrideMotorMode::kSpeedMode;
        SetMotorMode();
        EnableMotor();
    }
    SetSingleParam(RobstrideParamIdx::limit_cur, limit_current_);
    SetSingleParam(RobstrideParamIdx::acc_rad, acc_rad_);
    SetSingleParam(RobstrideParamIdx::spd_ref, speed_ref_);
}

/**
 * @brief          发送电流控制模式电机控制指令
 * @param[in]      robstride_motor:灵足电机结构体指针
 * @retval         无
 */
void Robstride::CurrentControl(void)
{
    if (run_mode_ != RobstrideMotorMode::kCurrentMode)
    {
        run_mode_ = RobstrideMotorMode::kCurrentMode;
        SetMotorMode();
        EnableMotor();
    }
    SetSingleParam(RobstrideParamIdx::iq_ref, iq_ref_);
}

/**
 * @brief          发送回零位控制指令
 * @param[in]      robstride_motor:灵足电机结构体指针
 * @retval         无
 */
void Robstride::GoZeroPosMode(void)
{
    run_mode_ = RobstrideMotorMode::kGoZeroPosMode;
    SetMotorMode();
}

/**
 * @brief          被回调函数调用,接收电机数据
 * @param[in]      can_ext_id: 已接收的扩展帧ID, can_rxdata: 接收到的数据数组
 * @retval         none
 */
void Robstride::CanRxCallBack(uint32_t can_ext_id, const uint8_t *can_rxdata)
{
    uint8_t robstride_can_id = 0;
    uint8_t com_type = 0;

    com_type = (can_ext_id & 0x1F000000) >> 24;
    robstride_can_id = (can_ext_id & 0xFF00) >> 8;

    if (robstride_can_id != can_id_)
    {
        return;
    }
    switch (com_type)
    {
        case kComTypeObtainDeviceID:
            ObtainDeviceIDReceive(can_rxdata);
            break;
        case kComTypeStatusFeedback:
            if (status_feedback_cnt_ == 255)
            {
                status_feedback_cnt_ = 11;
            }
            else
            {
                status_feedback_cnt_++;
            }
            StatusFeedbackReceive(can_ext_id, can_rxdata);
            break;
        case kComTypeReadSingleParam:
            ReadSingleParamReceive(can_ext_id, can_rxdata);
            break;
        case kComTypeFaultFeedback:
            FaultFeedbackReceive(can_rxdata);
            break;
        case kComTypeStatusFeedbackAuto:
            StatusFeedbackAutoReceive(can_ext_id, can_rxdata);
            break;
        default:
            break;
    }
}
