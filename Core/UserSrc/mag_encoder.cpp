#include "mag_encoder.hpp"
#include "gpio.h"
#include "utils.h"

/** DMA 发送缓冲区, 预填充了 Modbus 读保持寄存器请求帧
 *  帧结构: 0x01(地址) | 0x03(功能码) | 0x00 0x00(起始地址) | 0x00 0x02(寄存器数量) | 0xC4 0x0B(CRC16)
 */
__attribute__((section(".dma_buf"), aligned(32))) uint8_t g_mag_encoder_tx_dma_buf[2][32] = {
    {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xc4, 0x0B},
    {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xc4, 0x0B}
};
/* DMA 接收缓冲区, 用于接收编码器的 Modbus 应答帧(最大32字节) */
__attribute__((section(".dma_buf"), aligned(32))) uint8_t g_mag_encoder_rx_dma_buf[2][32] = {0};

uint8_t MagEncoder::instance_count = 0; /*!< 静态实例计数器, 用于分配 DMA 缓冲区索引 */

/**
 * @brief   磁栅尺编码器构造函数
 * @param   huart   UART 外设句柄引用
 * @details 从静态 DMA 缓冲区数组中为当前实例分配独立的收发缓冲区。
 *          最多支持 2 个实例, 超过则无缓冲区可用(rx_buffer_ 和 tx_buffer_ 保持 nullptr)
 * @note    DMA 缓冲区使用 ".dma_buf" 段以保证 32 字节对齐, 满足 DMA MPU 要求
 */
MagEncoder::MagEncoder(UART_HandleTypeDef &huart) : huart_(huart)
{
    if (instance_count < 2)
    {
        rx_buffer_ = g_mag_encoder_rx_dma_buf[instance_count];
        tx_buffer_ = g_mag_encoder_tx_dma_buf[instance_count];
        instance_count++;
    }
}

/**
 * @brief   发送 Modbus 读取请求, 启动 DMA 接收
 * @details 通信流程:
 *          1. 检查当前状态: 若处于 kWaitingForData 且未超时(20ms), 拒绝重复请求
 *             若已超时, 强行中止 DMA 接收并回退到 kIdle
 *          2. 清除 UART 溢出和 IDLE 标志, 启动 DMA 接收(ReceiveToIdle, 最大32字节)
 *          3. 通过 DMA 发送 8 字节 Modbus 请求帧
 *          4. 记录请求时刻, 切换到 kWaitingForData 等待应答
 * @note    Modbus 请求帧: 0x01(地址) | 0x03(功能码) | 0x0000(起始地址) | 0x0002(2个寄存器) | CRC16
 *          每次请求读取 2 个保持寄存器(共32位), 即编码器的绝对位置值
 */
void MagEncoder::SendRequest()
{
    /* 若正在等待应答且已超时, 强行回收资源 */
    if (state_ == State::kWaitingForData)
    {
        if (GetSysTimeMs() - request_start_ms_ > kTimeoutMs)
        {
            HAL_UART_AbortReceive(&huart_); /* 中止 DMA 接收, 防止缓冲区占用 */
            state_ = State::kIdle;
        }
        else
        {
            return; /* 未超时, 仍在等待应答, 不接受新请求 */
        }
    }

    if (state_ != State::kIdle) return; /* 状态异常, 拒绝发送 */

    /* 清除 UART 溢出和空闲标志, 为新的 DMA 接收做准备 */
    __HAL_UART_CLEAR_OREFLAG(&huart_);
    __HAL_UART_CLEAR_IDLEFLAG(&huart_);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart_, rx_buffer_, 32); /* 启动 DMA 接收, 收到 IDLE 中断时停止 */

    /* 填充 Modbus 读保持寄存器请求帧(8字节)并 DMA 发送 */
    tx_buffer_[0] = 0x01; /* 设备地址 */
    tx_buffer_[1] = 0x03; /* 功能码: 读保持寄存器 */
    tx_buffer_[2] = 0x00; /* 起始地址高字节 */
    tx_buffer_[3] = 0x00; /* 起始地址低字节 */
    tx_buffer_[4] = 0x00; /* 寄存器数量高字节 */
    tx_buffer_[5] = 0x02; /* 寄存器数量低字节(读取2个寄存器=32bit) */
    tx_buffer_[6] = 0xc4; /* CRC16 低字节(预计算) */
    tx_buffer_[7] = 0x0B; /* CRC16 高字节(预计算) */
    HAL_UART_Transmit_DMA(&huart_, tx_buffer_, 8);
    request_start_ms_ = GetSysTimeMs(); /* 记录发送时刻, 用于超时判断 */

    state_ = State::kWaitingForData; /* 进入等待应答状态 */
}

/**
 * @brief   UART 接收回调函数, 在 UART IDLE 中断或 DMA 半满中断时调用
 * @param   huart     触发中断的 UART 句柄
 * @param   data      接收到的数据指针
 * @param   data_size 接收到的数据长度
 * @details 解析 Modbus-RTU 应答帧:
 *          1. 校验基本条件: 数据非空、长度=9字节、UART 句柄匹配
 *          2. CRC16 校验: 对 data[0..6] 计算 CRC, 与 data[7..8] 比较
 *          3. 提取 data[3..6] 作为 32 位位置原始值(微米)
 *          4. 转换为毫米: absolute_position_mm_ = raw * 0.001
 *          5. 恢复 kIdle 状态, 允许下一次请求
 * @note    Modbus 应答帧(9字节): addr|func|byte_cnt|data(4B)|CRC16(2B)
 *          如果 CRC 校验失败, 数据被丢弃, 状态保持 kWaitingForData 等待超时重试
 */
void MagEncoder::UartRxCallback(UART_HandleTypeDef *huart, const uint8_t* data, uint16_t data_size)
{
    /* 基本校验: 数据有效性、长度、UART 句柄匹配 */
    if (data == nullptr || data_size != 9 || huart != &huart_) return;

    /* CRC16 校验: 计算 data[0..6] 的 CRC 值, 与接收到的 CRC(data[7..8]) 比较 */
    uint16_t crc_received = data[8] << 8 | data[7]; /* 接收到的 CRC(注意字节序: 低字节在前) */
    uint16_t crc_calculated = Crc16Modbus(data, 7); /* 计算得到的 CRC */
    if (crc_received != crc_calculated) return;      /* CRC 不匹配, 丢弃此帧 */

    /* 提取位置数据: data[3..6] = 32位位置值(微米), 转换为毫米 */
    raw_position_reading_um_ = HexArrayToDec(data, data_size);
    absolute_position_mm_ = (float)raw_position_reading_um_ * kUm2Mm;
    state_ = State::kIdle; /* 成功接收, 恢复到空闲状态, 允许下一次请求 */
}

/**
 * @brief   Modbus CRC-16 校验算法
 * @param   data      待计算的数据指针
 * @param   data_size 数据长度(字节数)
 * @return  16 位 CRC 校验值
 * @details 采用 Modbus 标准 CRC-16 算法:
 *          - 初始值: 0xFFFF
 *          - 多项式: 0xA001(即 0x8005 的反转)
 *          - 逐字节异或后按位处理, LSB 为 1 时右移并异或多项式
 */
uint16_t MagEncoder::Crc16Modbus(const uint8_t *data, uint16_t data_size)
{
    uint16_t crc = 0xFFFF; /* CRC 初始值 */
    uint16_t i, j;
    for (i = 0; i < data_size; i++)
    {
        crc ^= data[i]; /* 当前字节与 CRC 低字节异或 */

        for (j = 0; j < 8; j++) /* 逐位处理 */
        {
            if (crc & 0x0001)   /* LSB 为 1: 右移1位并异或多项式 */
            {
                crc >>= 1;
                crc ^= 0xA001;  /* 反转的多项式 0xA001 */
            }
            else                /* LSB 为 0: 仅右移1位 */
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}


/**
 * @brief   将 Modbus 应答帧中的 4 字节数据区转换为 32 位无符号整数
 * @param   hex_array Modbus 应答帧完整数据(至少 7 字节: addr|func|byte_cnt|data[4])
 * @param   length   数据长度
 * @return  32 位位置原始值(微米)
 * @details 从 hex_array[3..6] 提取 4 字节位置数据, 按大端序拼接为 uint32_t
 *          hex_array[3] 为最高字节(MSB), hex_array[6] 为最低字节(LSB)
 */
uint32_t MagEncoder::HexArrayToDec(const uint8_t* hex_array, uint8_t size)
{
    if (size < 7) return 0; /* 数据不足(至少需要 addr+func+byte_cnt+4字节数据) */

    uint32_t result = 0;
    uint8_t ch[4] = {hex_array[3], hex_array[4], hex_array[5], hex_array[6]}; /* 提取位置数据 4 字节 */
    for(uint8_t i = 0; i < 4; i++)
    {
        result = (result << 8) | ch[i]; /* 大端序拼接: MSB 先出现 */
    }
    return result;
}
