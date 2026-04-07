#include "dm_motor.hpp"
#include "fdcan.h"
#include <string.h>
#include <stdio.h>

/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
static int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}

/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
  * @brief          达妙电机数据发送函数
  * @param[in]      can_ext_id:CAN标准ID
  * @param[in]      data:待发送数据指针
  * @param[in]      data_size:待发送数据长度
  * @retval         无
  */
static void DMSendData(uint32_t can_std_id, uint8_t *data, uint32_t data_size)
{
    FDCanSendData(&hfdcan1, can_std_id, FDCAN_STANDARD_ID, data, data_size);
}

DMMotor::DMMotor(uint16_t can_id)
{
    can_id_ = can_id;
    mst_id_ = 0xFF - can_id;
    feedback_.flag_ = 0;
    ctrl_param_.mode_id_ = DMMotorModeID::kMIT;
    ctrl_param_.pos_set_rad_ = 0.0f;
    ctrl_param_.vel_set_radps_ = 0.0f;
    ctrl_param_.cur_set_A_ = 0.0f;
    ctrl_param_.tor_set_Nm_ = 0.0f;
    ctrl_param_.kp_set_  = 0.0f;
    ctrl_param_.kd_set_  = 0.0f;
    inf_.read_reg_ = DMMotorReg::RID_CMODE;
    inf_.PMAX_ = 12.5f;
    inf_.VMAX_ = 45.0f;
    inf_.TMAX_ = 12.0f;

}

void DMMotor::EnableMotor(void)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    uint16_t send_id = can_id_ + static_cast<uint16_t>(ctrl_param_.mode_id_);
    DMSendData(send_id, data, 8);
}

void DMMotor::DisableMotor(void)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    uint16_t send_id = can_id_ + static_cast<uint16_t>(ctrl_param_.mode_id_);
    DMSendData(send_id, data, 8);
}

void DMMotor::ClearError(void)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
    uint16_t send_id = can_id_ + static_cast<uint16_t>(ctrl_param_.mode_id_);
    DMSendData(send_id, data, 8);
}

void DMMotor::SetMecPosZero(void)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    uint16_t send_id = can_id_ + static_cast<uint16_t>(ctrl_param_.mode_id_);
    DMSendData(send_id, data, 8);
}

void DMMotor::SetMotorMode(void)
{
    uint8_t data[4] = {0};
    switch (ctrl_param_.mode_id_)
    {
    case DMMotorModeID::kMIT:
        data[0] = 0x01;
        break;
    case DMMotorModeID::kPosVel:
        data[0] = 0x02;
        break;
    case DMMotorModeID::kVel:
        data[0] = 0x03;
        break;
    case DMMotorModeID::kPosVelCur:
        data[0] = 0x04;
        break;
    default:
        break;
    }
    WriteReg(DMMotorReg::RID_CMODE, data);
}

void DMMotor::MitControl(void)
{
    uint8_t data[8] = {0};
    if (ctrl_param_.mode_id_ != DMMotorModeID::kMIT)
    {
        DisableMotor();
        ctrl_param_.mode_id_ = DMMotorModeID::kMIT;
        SetMotorMode();
        EnableMotor();
    }
    uint16_t send_id = can_id_ + static_cast<uint16_t>(DMMotorModeID::kMIT);

    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
	pos_tmp = float_to_uint(ctrl_param_.pos_set_rad_, -inf_.PMAX_, inf_.PMAX_, 16);
	vel_tmp = float_to_uint(ctrl_param_.vel_set_radps_, -inf_.VMAX_, inf_.VMAX_, 12);
	tor_tmp = float_to_uint(ctrl_param_.tor_set_Nm_, -inf_.TMAX_, inf_.TMAX_, 12);
	kp_tmp  = float_to_uint(ctrl_param_.kp_set_,  DM_KP_MIN, DM_KP_MAX, 12);
	kd_tmp  = float_to_uint(ctrl_param_.kd_set_,  DM_KD_MIN, DM_KD_MAX, 12);

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	data[7] = tor_tmp;

    DMSendData(send_id, data, 8);
}

void DMMotor::PosVelControl(void)
{
    uint8_t data[8] = {0};
    if (ctrl_param_.mode_id_ != DMMotorModeID::kPosVel)
    {
        DisableMotor();
        ctrl_param_.mode_id_ = DMMotorModeID::kPosVel;
        SetMotorMode();
        EnableMotor();
    }
    uint16_t send_id = can_id_ + static_cast<uint16_t>(DMMotorModeID::kPosVel);

    memcpy(&data[0], &ctrl_param_.pos_set_rad_, 4);
	memcpy(&data[4], &ctrl_param_.vel_set_radps_, 4);

    DMSendData(send_id, data, 8);
}

void DMMotor::VelControl(void)
{
    uint8_t data[8] = {0};
    if (ctrl_param_.mode_id_ != DMMotorModeID::kVel)
    {
        DisableMotor();
        ctrl_param_.mode_id_ = DMMotorModeID::kVel;
        SetMotorMode();
        EnableMotor();
    }
    uint16_t send_id = can_id_ + static_cast<uint16_t>(DMMotorModeID::kVel);

    memcpy(&data[0], &ctrl_param_.vel_set_radps_, 4);

    DMSendData(send_id, data, 4);
}

void DMMotor::PosVelCurControl(void)
{
    uint8_t data[8] = {0};
    if (ctrl_param_.mode_id_ != DMMotorModeID::kPosVelCur)
    {
        DisableMotor();
        ctrl_param_.mode_id_ = DMMotorModeID::kPosVelCur;
        SetMotorMode();
        EnableMotor();
    }
    uint16_t send_id = can_id_ + static_cast<uint16_t>(DMMotorModeID::kPosVelCur);

    uint16_t u16_vel = ctrl_param_.vel_set_radps_ * 100;
    uint16_t u16_cur = ctrl_param_.cur_set_A_ * 10000;
    uint8_t *vel_tmp = (uint8_t*)&u16_vel;
    uint8_t *cur_tmp = (uint8_t*)&u16_cur;

    memcpy(&data[0], &ctrl_param_.pos_set_rad_, 4);
    memcpy(&data[4], vel_tmp, 2);
    memcpy(&data[6], cur_tmp, 2);

    DMSendData(send_id, data, 8);
}

void DMMotor::ClearCtrlParam(void)
{
    // ctrl_param_.mode_id_ = DMMotorModeID::kMIT;
    ctrl_param_.pos_set_rad_ = 0.0f;
    ctrl_param_.vel_set_radps_ = 0.0f;
    ctrl_param_.tor_set_Nm_ = 0.0f;
    ctrl_param_.cur_set_A_ = 0.0f;
    ctrl_param_.kp_set_ = 0.0f;
    ctrl_param_.kd_set_ = 0.0f;
}

/** #TODO：检查can_id_low和can_id_high */
void DMMotor::ReadReg(DMMotorReg reg)
{
    uint8_t can_id_low = can_id_ & 0xFF;
    uint8_t can_id_high = (can_id_ >> 8) & 0x07;
    uint8_t data[4] = {can_id_low, can_id_high, 0x33, static_cast<uint8_t>(reg)};
    DMSendData(0x7FF, data, 4);
}

void DMMotor::WriteReg(DMMotorReg reg, uint8_t value[4])
{
    uint8_t can_id_low = can_id_ & 0xFF;
    uint8_t can_id_high = (can_id_ >> 8) & 0x07;

    uint8_t data[8] = {can_id_low, can_id_high, 0x55, static_cast<uint8_t>(reg), value[0], value[1], value[2], value[3]};
    DMSendData(0x7FF, data, 8);
}

void DMMotor::ReadFeedback(void)
{
    uint8_t can_id_low = can_id_ & 0xFF;
    uint8_t can_id_high = (can_id_ >> 8) & 0x07;

    uint8_t data[4] = {can_id_low, can_id_high, 0xCC, 0x00};
    DMSendData(0x7FF, data, 4);
}

void DMMotor::SaveToFlash(void)
{
	uint8_t can_id_low = can_id_ & 0xFF;       // 低 8 位
    uint8_t can_id_high = (can_id_ >> 8) & 0x07; // 高 3 位
	
	uint8_t data[4] = {can_id_low, can_id_high, 0xAA, 0x01};
    DMSendData(0x7FF, data, 4);
}

void DMMotor::CanRxCallBack(uint32_t can_id, uint8_t *can_rxdata)
{
    if (can_id != (uint32_t)mst_id_)
    {
        return;
    }

    if (can_rxdata[0] == (can_id_ & 0xFF) && can_rxdata[1] == ((can_id_ >> 8) & 0xFF) && can_rxdata[2] == 0x33)
    {
        feedback_.flag_ = 1;
        uint8_t bytes[4] = {can_rxdata[4],can_rxdata[5],can_rxdata[6],can_rxdata[7]};
        float *ptr_float_data = (float*)bytes;
        uint32_t *ptr_uint32_data = (uint32_t*)bytes;

        DMMotorReg reg = static_cast<DMMotorReg>(can_rxdata[3]);
        switch (reg)
        {
        case DMMotorReg::RID_UV_VALUE:
            inf_.UV_Value_ = *ptr_float_data;
            break;
        case DMMotorReg::RID_KT_VALUE:
            inf_.KT_Value_ = *ptr_float_data;
            break;
        case DMMotorReg::RID_OT_VALUE:
            inf_.OT_Value_ = *ptr_float_data;
            break;
        case DMMotorReg::RID_OC_VALUE:
            inf_.OC_Value_ = *ptr_float_data;
            break;
        case DMMotorReg::RID_ACC:
            inf_.ACC_ = *ptr_float_data;
            break;
        case DMMotorReg::RID_DEC:
            inf_.DEC_ = *ptr_float_data;
            break;
        case DMMotorReg::RID_MAX_SPD:
            inf_.MAX_SPD_ = *ptr_float_data;
            break;
        case DMMotorReg::RID_MST_ID:
            inf_.MST_ID_ = *ptr_uint32_data;
            break;
        case DMMotorReg::RID_ESC_ID:
            inf_.ESC_ID_ = *ptr_uint32_data;
            break;
        case DMMotorReg::RID_TIMEOUT:
            inf_.TIMEOUT_ = *ptr_uint32_data;
            break;
        case DMMotorReg::RID_CMODE:
            inf_.cmode_ = *ptr_uint32_data;
            break;
        case DMMotorReg::RID_DAMP:
            inf_.Damp_ = *ptr_float_data;
            break;
        case DMMotorReg::RID_INERTIA:
            inf_.Inertia_ = *ptr_float_data;
            break;
        case DMMotorReg::RID_HW_VER:
            inf_.hw_ver_ = *ptr_uint32_data;
            break;
        case DMMotorReg::RID_SW_VER:
            inf_.sw_ver_ = *ptr_uint32_data;
            break;
        case DMMotorReg::RID_SN:
            inf_.SN_ = *ptr_uint32_data;
            break;
        case DMMotorReg::RID_NPP:
            inf_.NPP_ = *ptr_uint32_data;
            break;
        case DMMotorReg::RID_RS:
            inf_.Rs_ = *ptr_float_data;
            break;
        case DMMotorReg::RID_LS:
            inf_.Ls_ = *ptr_float_data;
            break;
        case DMMotorReg::RID_FLUX:
            inf_.Flux_ = *ptr_float_data;
            break;
        case DMMotorReg::RID_GR:
            inf_.Gr_ = *ptr_float_data;
            break;
        case DMMotorReg::RID_PMAX:
            inf_.PMAX_ = *ptr_float_data;
            break;
        case DMMotorReg::RID_VMAX:
            inf_.VMAX_ = *ptr_float_data;
            break;
        case DMMotorReg::RID_TMAX:
            inf_.TMAX_ = *ptr_float_data;
            break;
        case DMMotorReg::RID_I_BW:
            inf_.I_BW_ = *ptr_float_data;
            break;
        case DMMotorReg::RID_KP_ASR:
            inf_.KP_ASR_ = *ptr_float_data;
            break;
        case DMMotorReg::RID_KI_ASR:
            inf_.KI_ASR_ = *ptr_float_data;
            break;
        case DMMotorReg::RID_KP_APR:
            inf_.KP_APR_ = *ptr_float_data;
            break;
        case DMMotorReg::RID_KI_APR:
            inf_.KI_APR_ = *ptr_float_data;
            break;
        case DMMotorReg::RID_OV_VALUE:
            inf_.OV_Value_ = *ptr_float_data;
            break;
        case DMMotorReg::RID_GREF:
            inf_.GREF_ = *ptr_float_data;
            break;
        case DMMotorReg::RID_DETA:
            inf_.Deta_ = *ptr_float_data;
            break;
        case DMMotorReg::RID_V_BW:
            inf_.V_BW_ = *ptr_float_data;
            break;
        case DMMotorReg::RID_IQ_CL:
            inf_.IQ_cl_    = *ptr_float_data; 
            break;
        case DMMotorReg::RID_VL_CL:
            inf_.VL_cl_    = *ptr_float_data;
            break;
        case DMMotorReg::RID_CAN_BR:
            inf_.can_br_   = *ptr_uint32_data;
            break;
        case DMMotorReg::RID_SUB_VER:
            inf_.sub_ver_  = *ptr_uint32_data;
            break;
        case DMMotorReg::RID_U_OFF:
            inf_.u_off_    = *ptr_float_data;
            break;
        case DMMotorReg::RID_V_OFF:
            inf_.v_off_    = *ptr_float_data;
            break;
        case DMMotorReg::RID_K1:
            inf_.k1_       = *ptr_float_data;
            break;
        case DMMotorReg::RID_K2:
            inf_.k2_       = *ptr_float_data;
            break;
        case DMMotorReg::RID_M_OFF:
            inf_.m_off_    = *ptr_float_data;
            break;
        case DMMotorReg::RID_DIR:
            inf_.dir_      = *ptr_float_data;
            break;
        case DMMotorReg::RID_P_M:
            inf_.p_m_      = *ptr_float_data;
            break;
        case DMMotorReg::RID_X_OUT:
            inf_.x_out_    = *ptr_float_data;
            break;
        default:
            break;
        }
    }
    else 
    {
        feedback_.flag_ = 1;
        /** #TODO: 检查 feedback_.id_ 的读取方式是否正确*/
        feedback_.id_ = can_rxdata[0] & 0x0F;
        feedback_.state_ = (can_rxdata[0] >> 4);
        int p_int_ = (can_rxdata[1] << 8) | can_rxdata[2];
        int v_int_ = (can_rxdata[3] << 4) | (can_rxdata[4] >> 4);
        int t_int_ = ((can_rxdata[4] & 0xF) << 8) | can_rxdata[5];
        feedback_.pos_rad_ = uint_to_float(p_int_, -inf_.PMAX_, inf_.PMAX_, 16);
        feedback_.vel_radps_ = uint_to_float(v_int_, -inf_.VMAX_, inf_.VMAX_, 12);
        feedback_.tor_Nm_ = uint_to_float(t_int_, -inf_.TMAX_, inf_.TMAX_, 12);
        feedback_.Tmos_ = (float)(can_rxdata[6]);
        feedback_.Tcoil_ = (float)(can_rxdata[7]);
    }

}


/* ------------------ C wrapper ------------------- */
void CallDMCanRxCallBack(struct DMMotor *ptr_dm, uint32_t can_id, uint8_t *rx_data)
{
    if (ptr_dm == nullptr || rx_data == nullptr)
    {
        return;
    }
    ptr_dm->CanRxCallBack(can_id, rx_data);
}