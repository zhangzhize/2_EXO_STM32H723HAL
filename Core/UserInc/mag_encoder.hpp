/**
 * @file    mag_encoder.hpp
 * @brief   磁栅尺编码器通信协议封装(Modbus-RTU over UART)
 *
 * @details 本文件定义了磁栅尺位置传感器的 UART 通信接口类。
 *          通信协议采用 Modbus-RTU 格式:
 *          - 请求帧: 0x01(地址) + 0x03(功能码: 读保持寄存器) + 起始地址(2B) + 寄存器数量(2B) + CRC16(2B)
 *          - 应答帧: 0x01(地址) + 0x03(功能码) + 字节数(1B) + 数据(N字节) + CRC16(2B)
 *          - CRC 校验采用 Modbus CRC-16 标准多项式: 0xA001
 *
 *          通信流程: SendRequest() 发送请求 -> 等待 UART IDLE 中断 -> UartRxCallback() 解析应答
 *          数据解析时提取 data[3..6] 共 4 字节作为 32 位位置值(微米), 转换为毫米存储
 *
 *          硬件使用 DMA 收发以降低 CPU 负载, 支持最多 2 个实例(使用静态分配的 DMA 缓冲区)
 * @note    目前通过静态 DMA 缓冲区实现最多 2 个实例, 超过会无缓冲区可用
 */
#ifndef MAG_ENCODER_HPP
#define MAG_ENCODER_HPP

#include <cstdint>
#include "usart.h"

/**
 * @class   MagEncoder
 * @brief   磁栅尺编码器类
 *
 * @details 通过 UART + Modbus-RTU 协议与磁栅尺编码器通信, 获取绝对位置读数。
 *          采用请求-应答模式: 调用 SendRequest() 发送 Modbus 读寄存器命令,
 *          编码器应答后由 UartRxCallback() 解析位置数据。
 *
 *          支持超时重传机制: 若在 kTimeoutMs(20ms) 内未收到应答, 则复位状态并允许重试。
 *          位置原始值单位为微米, 自动转换为毫米(乘以 kUm2Mm = 0.001)
 */
class MagEncoder
{
public:
    enum class State : uint8_t /** 通信状态机 */
    {
        kIdle,              /** 空闲状态, 可发送新请求 */
        kWaitingForData,    /** 等待编码器应答, 拒绝重复请求 */
    };

    explicit MagEncoder(UART_HandleTypeDef &uart);
    virtual ~MagEncoder() = default;

    void SendRequest(); /** 发送 Modbus 读保持寄存器请求, 启动 DMA 接收等待应答 */
    void UartRxCallback(UART_HandleTypeDef *uart, const uint8_t *data, uint16_t data_size); /** UART 接收回调, 校验 CRC 并解析位置数据 */

    float absolute_position_mm_ = 0.0f; /*!< 绝对位置(mm), 由 raw_position_reading_um_ * 0.001 转换 */
    static uint16_t Crc16Modbus(const uint8_t *data, uint16_t data_size); /** Modbus CRC-16 校验算法(多项式 0xA001) */
    static uint32_t HexArrayToDec(const uint8_t *hex_array, uint8_t length); /** 将 Modbus 应答数据区(4字节)转换为无符号32位整数 */
    State state_ = State::kIdle; /*!< 当前通信状态 */

    UART_HandleTypeDef &huart_; /*!< UART 外设句柄引用 */
private:

    uint8_t *rx_buffer_ = nullptr;  /*!< DMA 接收缓冲区指针(指向静态分配的全局数组) */
    uint8_t *tx_buffer_ = nullptr;  /*!< DMA 发送缓冲区指针(指向静态分配的全局数组) */

    static constexpr float kUm2Mm = 0.001f;       /*!< 微米到毫米的转换系数 */
    static constexpr uint32_t kTimeoutMs = 20;     /*!< 应答超时时间(ms), 超时后允许重试 */
    uint32_t raw_position_reading_um_ = 0;         /*!< 原始位置读数(微米), 从 Modbus 应答中提取 */
    uint32_t request_start_ms_ = 0;                /*!< 请求发送时刻的系统时间(ms), 用于超时判断 */

    static uint8_t instance_count;                 /*!< 实例计数器(最多2个), 用于分配 DMA 缓冲区 */

    bool is_first_reading_ = true;                 /*!< 首次读取标志(保留, 可用于初始化判断) */
};

#endif