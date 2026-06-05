#include "dji_esc.hpp"
#include "utils.h"

/**
 * @brief   统一发送所有电调的电流指令
 * @details 将 8 台电调的 Iq 参考值打包为两帧 CAN 标准帧:
 *          - ID=0x200: 电调1~4 的电流指令, 每电调 2 字节大端序 int16
 *          - ID=0x1FF: 电调5~8 的电流指令, 同样的 2 字节编码
 *          仅当对应电调处于活跃状态(is_active_)时才发送, 避免无意义的空帧
 * @note    调用此函数前需确保所有电调都已调用过控制函数并 SetTxIqRef
 */
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

    /* 检查电调1~4是否有活跃的(有新数据需要发送) */
    bool need_send_200 = is_active_[0] || is_active_[1] || is_active_[2] || is_active_[3];
    /* 检查电调5~8是否有活跃的 */
    bool need_send_1FF = is_active_[4] || is_active_[5] || is_active_[6] || is_active_[7];

    if (need_send_200)
    {
        uint8_t data_200[8] = {0};
        for (int i = 0; i < 4; i++)
        {
            /* 每电调2字节, 大端序 int16 */
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
            /* 索引映射: 电调5~8 -> data[0..7] */
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

/**
 * @brief   大疆电调构造函数
 * @param   hub        所属 CAN 集线器
 * @param   esc_id     电调 ID(1~8)
 * @param   motor_type 电机类型, 决定转矩常数/最大电流/减速比等参数
 * @details 根据电机类型初始化物理参数(kt, max_iq, reduction_ratio)。
 *          PID 控制器的输出限幅设置为最大电流值,
 *          因为 speed_pid_ 的输出直接作为 Iq 参考通过 CAN 发送。
 *          构造函数末尾自动调用 hub_.RegisterNode() 注册到集线器。
 * @note    M3508 的减速箱已拆除, reduction_ratio 设为 1.0
 */
DjiEsc::DjiEsc(DjiEscHub& hub, EscId esc_id, MotorType motor_type)
    : pos_pid_(46.0f, 20.0f, 0.0f, -500.0f, 10000.0f),
    speed_pid_(0.1f, 0.02f, 0.0f, -100.0f, motor_type == MotorType::kM3508 ? 20.0f : 10.0f),
    esc_id_(esc_id),
    motor_type_(motor_type),
    hub_(hub)
{
    if (motor_type == MotorType::kM3508)
    {
        kt_NmpA = 0.3f;              /* M3508 转矩常数: 0.3 Nm/A */
        max_iqref_amp_ = 20.0f;      /* C620 电调最大电流: 20A */
        // reduction_ratio_ = 3591.0f / 187.0f;
        reduction_ratio_ = 1.0f;     /* 减速箱已拆除, 减速比设为 1:1 */
        max_iqref_to_can_data_ = 16384; /* 20A 对应的 CAN 数据值 */

    }
    else if (motor_type == MotorType::kM2006)
    {
        kt_NmpA = 0.18f;             /* M2006 转矩常数: 0.18 Nm/A */
        max_iqref_amp_ = 10.0f;      /* C610 电调最大电流: 10A */
        reduction_ratio_ = 36.0f;    /* M2006 减速比: 36:1 */
        max_iqref_to_can_data_ = 10000; /* 10A 对应的 CAN 数据值 */
    }

    hub_.RegisterNode(static_cast<uint8_t>(esc_id_), this); /* 自动注册到集线器 */
}

/**
 * @brief   CAN 反馈数据解析, 由 DjiEscHub::CanRxCallBack() 自动调用
 * @param   rx_data  接收到的 8 字节数据区
 * @details 大疆电调反馈帧(0x201~0x208)的数据布局:
 *          data[0..1] = 编码器位置(0~8191), 大端序 uint16
 *          data[2..3] = 转子转速(rpm), 大端序 int16
 *          data[4..5] = 实际电流(raw), 大端序 int16
 *          data[6..7] = 保留(未使用)
 *
 *          位置处理: 编码器值映射到 0~2pi, 通过前后两次圈内位置的跳变检测过零点,
 *          累计转子圈数, 最终输出轴位置 = 转子绝对位置 / 减速比
 *          速度处理: RPM 转 rad/s, 再除以减速比得到输出轴角速度
 *          电流处理: 按比例换算, raw/max_iqref_to_can_data * max_iqref_amp = 实际电流
 */
void DjiEsc::UpdateFeedback(const uint8_t *rx_data)
{
    if (rx_data == nullptr) return;

    /* 1. 物理位置解析 — 编码器值 -> 圈内角度 -> 累计圈数 -> 输出轴绝对位置 */
    rotor_last_pos_one_round_rad_ = rotor_pos_one_round_rad_; /* 保存上次圈内角度, 用于过零检测 */
    uint16_t raw_pos = static_cast<uint16_t>((rx_data[0] << 8) | rx_data[1]);
    rotor_pos_one_round_rad_ = _2PI * static_cast<float>(raw_pos) / kEncoderCountsPerRev; /* 当前圈内角度 0~2pi */

    /* 过零检测: 若相邻两次位置差 > pi, 说明转子跨越了编码器零点(反向), 圈数减1 */
    if (rotor_pos_one_round_rad_ - rotor_last_pos_one_round_rad_ > _PI)
    {
        rotor_round_count_--;
    }
    /* 若差值 < -pi, 说明转子正向跨越零点, 圈数加1 */
    else if (rotor_pos_one_round_rad_ - rotor_last_pos_one_round_rad_ < -_PI)
    {
        rotor_round_count_++;
    }
    float rotor_pos_abs_rad = static_cast<float>(rotor_round_count_) * _2PI + rotor_pos_one_round_rad_; /* 转子累计绝对角度 */
    shaft_pos_feedback_rad_ = rotor_pos_abs_rad / reduction_ratio_;     /** 输出轴角度 = 转子总角度 / 减速比 */

    /* 2. 物理速度解析 — RPM -> rad/s -> 输出轴速度 */
    int16_t raw_rpm = static_cast<int16_t>((rx_data[2] << 8) | rx_data[3]);
    float rotor_speed_radps = static_cast<float>(raw_rpm) * RPM_TO_RADPS; /* 转子角速度 rad/s */
    shaft_speed_feedback_radps_ = rotor_speed_radps / reduction_ratio_; /** 输出轴速度 = 转子速度 / 减速比 */

    /* 3. 电流解析 — raw 值按比例映射到实际电流, 再换算为扭矩 */
    int16_t raw_current = static_cast<int16_t>((rx_data[4] << 8) | rx_data[5]);
    rotor_iq_feedback_amp_ = static_cast<float>(raw_current) * max_iqref_amp_ / static_cast<float>(max_iqref_to_can_data_);

    shaft_torque_feedback_Nm_ = rotor_iq_feedback_amp_ * kt_NmpA * reduction_ratio_; /** 输出轴扭矩 = iq * kt * ratio (暂不考虑效率) */
}

/* 位置控制, 在调用前需要设置位置参考值shaft_pos_reference_rad_ (减速器输出端); 最后实际发送的时候需要再调用SendAllCanTxData() */
void DjiEsc::PositionControl()
{
    if (mode_ != EscMode::kPosition)
    {
        pos_pid_.ResetError();
        speed_pid_.ResetError();
        mode_ = EscMode::kPosition;
    }

    float shaft_pos_error_rad = shaft_pos_reference_rad_ - shaft_pos_feedback_rad_;
    shaft_speed_reference_radps_ = pos_pid_(shaft_pos_error_rad);

    float shaft_speed_error_radps = shaft_speed_reference_radps_ - shaft_speed_feedback_radps_;
    rotor_iq_reference_amp_ = speed_pid_(shaft_speed_error_radps);

    int16_t iq_tx_data = static_cast<int16_t>(rotor_iq_reference_amp_ * static_cast<float>(max_iqref_to_can_data_) / max_iqref_amp_);
    hub_.SetTxIqRef(static_cast<uint8_t>(esc_id_), iq_tx_data);
}

/* 速度控制, 在调用前需要设置速度参考值shaft_speed_reference_radps_ (减速器输出端); 最后实际发送的时候需要再调用SendAllCanTxData() */
void DjiEsc::SpeedControl()
{
    if (mode_ != EscMode::kSpeed)
    {
        pos_pid_.ResetError();
        speed_pid_.ResetError();
        mode_ = EscMode::kSpeed;
    }

    float speed_error_radps = shaft_speed_reference_radps_ - shaft_speed_feedback_radps_;
    rotor_iq_reference_amp_ = speed_pid_(speed_error_radps);

    int16_t iq_tx_data = static_cast<int16_t>(rotor_iq_reference_amp_ * static_cast<float>(max_iqref_to_can_data_) / max_iqref_amp_);
    hub_.SetTxIqRef(static_cast<uint8_t>(esc_id_), iq_tx_data);
}

/* 电流控制, (很少用到) 在调用前需要设置电流参考值rotor_iq_reference_amp_; 最后实际发送的时候需要再调用SendAllCanTxData() */
void DjiEsc::CurrentControl()
{
    if (mode_ != EscMode::kCurrent)
    {
        pos_pid_.ResetError();
        speed_pid_.ResetError();
        mode_ = EscMode::kCurrent;
    }

    /* 前面位置和速度控制的PID会自己限幅 但这里没有, 所以需要手动限幅 */
    rotor_iq_reference_amp_ = _constrain(rotor_iq_reference_amp_, -max_iqref_amp_, max_iqref_amp_);
    int16_t iq_tx_data = static_cast<int16_t>(rotor_iq_reference_amp_ * static_cast<float>(max_iqref_to_can_data_) / max_iqref_amp_);
    hub_.SetTxIqRef(static_cast<uint8_t>(esc_id_), iq_tx_data);
}

/* 大疆电调无使能指令, 这里就简单的令iqref=0 */
void DjiEsc::EnableMotor()
{
    int16_t iq_tx_data = 0;
    hub_.SetTxIqRef(static_cast<uint8_t>(esc_id_), iq_tx_data);
    pos_pid_.ResetError();
    speed_pid_.ResetError();
}

/* 大疆电调无失能指令, 这里就简单的令iqref=0 */
void DjiEsc::DisableMotor()
{
    mode_ = EscMode::kStop;
    int16_t iq_tx_data = 0;
    hub_.SetTxIqRef(static_cast<uint8_t>(esc_id_), iq_tx_data);
    pos_pid_.ResetError();
    speed_pid_.ResetError();
}
