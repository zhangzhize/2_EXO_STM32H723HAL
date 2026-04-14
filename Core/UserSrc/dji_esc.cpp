#include "dji_esc.hpp"
#include "utils.h"

void DjiEscHub::SendAllCanTxData()
{
    FDCAN_TxHeaderTypeDef pTxHeader;
    pTxHeader.IdType = FDCAN_STANDARD_ID;
    pTxHeader.TxFrameType = FDCAN_DATA_FRAME;
    pTxHeader.DataLength = FDCAN_DLC_BYTES_8;
    pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    pTxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    pTxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    pTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker = 0;

    bool need_send_200 = is_active_[0] || is_active_[1] || is_active_[2] || is_active_[3];
    bool need_send_1FF = is_active_[4] || is_active_[5] || is_active_[6] || is_active_[7];

    if (need_send_200)
    {
        uint8_t data_200[8] = {0};
        for (int i = 0; i < 4; i++)
        {
            data_200[i*2]     = static_cast<uint8_t>(tx_iq_refs_[i] >> 8);
            data_200[i*2 + 1] = static_cast<uint8_t>(tx_iq_refs_[i] & 0xFF);
        }
        pTxHeader.Identifier = 0x200;
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan_, &pTxHeader, data_200);
    }

    if (need_send_1FF) 
    {
        uint8_t data_1FF[8] = {0};
        for (int i = 4; i < 8; i++)
        {
            data_1FF[(i-4)*2]     = static_cast<uint8_t>(tx_iq_refs_[i] >> 8);
            data_1FF[(i-4)*2 + 1] = static_cast<uint8_t>(tx_iq_refs_[i] & 0xFF);
        }
        pTxHeader.Identifier = 0x1FF;
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan_, &pTxHeader, data_1FF);
    }
}

void DjiEscHub::RegisterNode(uint8_t esc_id, DjiEsc *node)
{
    if (esc_id >= 1 && esc_id <= 8 && node != nullptr)
    {
        nodes_[esc_id - 1] = node;
        is_active_[esc_id - 1] = true;
    }
}

void DjiEscHub::CanRxCallBack(uint32_t can_std_id, const uint8_t *rx_data)
{
    if (can_std_id >= 0x201 && can_std_id <= 0x208)
    {
        uint8_t index = can_std_id - 0x201;
        if (nodes_[index] != nullptr)
        {
            nodes_[index]->UpdateFeedback(rx_data);
        }
    }
}

/** speed_pid_的输出是iqref, 那么其最后一个参数output_limit设置为参考电流的最大值 */
DjiEsc::DjiEsc(DjiEscHub& hub, EscId esc_id, MotorType motor_type)
    : position_pid_(40.0f, 25.0f, 0.0f, -500.0f, 10000.0f),
    speed_pid_(0.05f, 0.02f, 0.0f, -100.0f, motor_type == MotorType::kM3508 ? 20.0f : 10.0f),
    esc_id_(esc_id),
    motor_type_(motor_type),
    hub_(hub)
{
    if (motor_type == MotorType::kM3508)
    {
        kt_NmpA = 0.3f;
        max_iqref_amp_ = 20.0f;
        // reduction_ratio_ = 3591.0f / 187.0f;
        reduction_ratio_ = 1.0f; /** now the gear box has been removed */
        max_iqref_to_can_data_ = 16384;

    }
    else if (motor_type == MotorType::kM2006)
    {
        kt_NmpA = 0.18f;
        max_iqref_amp_ = 10.0f;
        reduction_ratio_ = 36.0f;
        max_iqref_to_can_data_ = 10000;
    }

    hub_.RegisterNode(static_cast<uint8_t>(esc_id_), this);
}

/** CAN接收回调函数, 解析反馈数据; 注意, 解析得到的是减速器输出端的位置和速度 */
void DjiEsc::UpdateFeedback(const uint8_t *rx_data)
{
    if (rx_data == nullptr) return;

    /** 1. 物理位置解析 */
    rotor_last_pos_one_round_rad_ = rotor_pos_one_round_rad_;
    uint16_t raw_pos = static_cast<uint16_t>((rx_data[0] << 8) | rx_data[1]);
    rotor_pos_one_round_rad_ = _2PI * static_cast<float>(raw_pos) / kEncoderCountsPerRev;

    if (rotor_pos_one_round_rad_ - rotor_last_pos_one_round_rad_ > _PI)
    {
        rotor_round_count_--;
    }
    else if (rotor_pos_one_round_rad_ - rotor_last_pos_one_round_rad_ < -_PI)
    {
        rotor_round_count_++;
    }
    float rotor_pos_abs_rad = static_cast<float>(rotor_round_count_) * _2PI + rotor_pos_one_round_rad_;
    shaft_pos_feedback_rad_ = rotor_pos_abs_rad / reduction_ratio_;     /** 输出轴角度 = 转子总角度 / 减速比 */

    /** 2. 物理速度解析 */
    int16_t raw_rpm = static_cast<int16_t>((rx_data[2] << 8) | rx_data[3]);
    float rotor_speed_radps = static_cast<float>(raw_rpm) * RPM_TO_RAD;
    shaft_speed_feedback_radps_ = rotor_speed_radps / reduction_ratio_; /** 输出轴速度 = 转子速度 / 减速比 */

    /** 3. 电流解析 */
    int16_t raw_current = static_cast<int16_t>((rx_data[4] << 8) | rx_data[5]);
    rotor_iq_feedback_amp_ = static_cast<float>(raw_current) * max_iqref_amp_ / static_cast<float>(max_iqref_to_can_data_);

    shaft_torque_feedback_Nm_ = rotor_iq_feedback_amp_ * kt_NmpA * reduction_ratio_; /** 暂不考虑效率 */
}

/** 位置控制, 在调用前需要设置位置参考值shaft_pos_reference_rad_ (减速器输出端); 最后实际发送的时候需要再调用SendAllCanTxData() */
void DjiEsc::PositionControl()
{
    if (mode_ != EscMode::kPosition)
    {
        position_pid_.ResetError();
        speed_pid_.ResetError();
        mode_ = EscMode::kPosition;
    }

    float shaft_pos_error_rad = shaft_pos_reference_rad_ - shaft_pos_feedback_rad_;
    shaft_speed_reference_radps_ = position_pid_(shaft_pos_error_rad);

    float shaft_speed_error_radps = shaft_speed_reference_radps_ - shaft_speed_feedback_radps_;
    rotor_iq_reference_amp_ = speed_pid_(shaft_speed_error_radps);

    int16_t iq_tx_data = static_cast<int16_t>(rotor_iq_reference_amp_ * static_cast<float>(max_iqref_to_can_data_) / max_iqref_amp_);
    hub_.SetTxIqRef(static_cast<uint8_t>(esc_id_), iq_tx_data);
}

/** 速度控制, 在调用前需要设置速度参考值shaft_speed_reference_radps_ (减速器输出端); 最后实际发送的时候需要再调用SendAllCanTxData() */
void DjiEsc::SpeedControl()
{
    if (mode_ != EscMode::kSpeed)
    {
        position_pid_.ResetError();
        speed_pid_.ResetError();
        mode_ = EscMode::kSpeed;
    }

    float speed_error_radps = shaft_speed_reference_radps_ - shaft_speed_feedback_radps_;
    rotor_iq_reference_amp_ = speed_pid_(speed_error_radps);

    int16_t iq_tx_data = static_cast<int16_t>(rotor_iq_reference_amp_ * static_cast<float>(max_iqref_to_can_data_) / max_iqref_amp_);
    hub_.SetTxIqRef(static_cast<uint8_t>(esc_id_), iq_tx_data);
}

/** 电流控制, (很少用到) 在调用前需要设置电流参考值rotor_iq_reference_amp_; 最后实际发送的时候需要再调用SendAllCanTxData() */
void DjiEsc::CurrentControl()
{
    if (mode_ != EscMode::kCurrent)
    {
        position_pid_.ResetError();
        speed_pid_.ResetError();
        mode_ = EscMode::kCurrent;
    }

    /** 前面位置和速度控制的PID会自己限幅 但这里没有, 所以需要手动限幅 */
    rotor_iq_reference_amp_ = _constrain(rotor_iq_reference_amp_, -max_iqref_amp_, max_iqref_amp_);
    int16_t iq_tx_data = static_cast<int16_t>(rotor_iq_reference_amp_ * static_cast<float>(max_iqref_to_can_data_) / max_iqref_amp_);
    hub_.SetTxIqRef(static_cast<uint8_t>(esc_id_), iq_tx_data);
}

/** 大疆电调无使能指令, 这里就简单的令iqref=0 */
void DjiEsc::EnableMotor()
{
    int16_t iq_tx_data = 0;
    hub_.SetTxIqRef(static_cast<uint8_t>(esc_id_), iq_tx_data);
    position_pid_.ResetError();
    speed_pid_.ResetError();
}

/** 大疆电调无失能指令, 这里就简单的令iqref=0 */
void DjiEsc::DisableMotor()
{
    mode_ = EscMode::kStop;
    int16_t iq_tx_data = 0;
    hub_.SetTxIqRef(static_cast<uint8_t>(esc_id_), iq_tx_data);
    position_pid_.ResetError();
    speed_pid_.ResetError();
}
