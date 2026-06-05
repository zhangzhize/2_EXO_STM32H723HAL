#include "bmi088_middleware.h"
#include "main.h"
#include "utils.h"

/* BMI088 使用的 SPI 总线单元 —— SPI2 */
#define BMI088_USING_SPI_UNIT   hspi2

extern SPI_HandleTypeDef BMI088_USING_SPI_UNIT;

/**
 * @brief  BMI088 GPIO 初始化（占位函数）
 *
 * @details 引脚配置由 CubeMX 生成的 MX_GPIO_Init() 完成，
 *          此处预留接口以保持驱动框架的完整性。
 */
void BMI088_GPIO_init(void)
{

}

/**
 * @brief  BMI088 通信接口初始化（占位函数）
 *
 * @details SPI 初始化由 CubeMX 生成的 MX_SPI2_Init() 完成，
 *          此处预留接口以保持驱动框架的完整性。
 */
void BMI088_com_init(void)
{


}

/**
 * @brief  毫秒级延时
 * @param  ms  延时毫秒数
 *
 * @details 调用系统延时函数 DelayMs()。
 *          zzz 修改：原实现使用 HAL_Delay()，改为使用 DWT 计时的 DelayMs()
 *          以获得更高精度且不依赖 SysTick 中断。
 */
void BMI088_delay_ms(uint16_t ms)
{
    /* zzz: 使用 DWT 精确延时替代 HAL_Delay() */
    DelayMs(ms);
    return;
}

/**
 * @brief  微秒级延时
 * @param  us  延时微秒数
 *
 * @details 调用系统延时函数 DelayUs()，底层使用 DWT 周期计数器。
 */
void BMI088_delay_us(uint16_t us)
{
    /* zzz: 使用 DWT 精确微秒延时 */
    DelayUs(us);
    return;
}

/**
 * @brief  选中加速度计 —— CS 拉低
 *
 * @details 拉低加速度计的片选引脚，使其进入 SPI 通信就绪状态。
 *          在 CS 为低期间，加速度计会响应 SPI 总线上的命令和数据。
 */
void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(BMI088_ACC_CS_GPIO_Port, BMI088_ACC_CS_Pin, GPIO_PIN_RESET);
}

/**
 * @brief  释放加速度计 —— CS 拉高
 *
 * @details 拉高加速度计的片选引脚，结束本次 SPI 通信。
 *          CS 的上升沿可能触发传感器内部的状态锁存。
 */
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(BMI088_ACC_CS_GPIO_Port, BMI088_ACC_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief  选中陀螺仪 —— CS 拉低
 */
void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_RESET);
}

/**
 * @brief  释放陀螺仪 —— CS 拉高
 */
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief  SPI 单字节全双工读写
 * @param  txdata  要发送的字节
 * @return 同时接收到的字节
 *
 * @details 利用 SPI 全双工特性，每发送一个字节的同时接收一个字节。
 *          超时设为 1000 个 tick（约 1000ms），在正常情况下的
 *          SPI 通信（几兆 Hz）中远超实际需要。
 */
uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&BMI088_USING_SPI_UNIT, &txdata, &rx_data, 1, 1000);
    return rx_data;
}

/**
 * @brief  加速度计 SPI 突发读取（zzz 优化）
 *
 * @param  reg  起始寄存器地址
 * @param  buf  接收数据缓冲区
 * @param  len  有效数据字节数
 *
 * @details 实现细节：
 *          1. 安全检查：len > 30 则直接返回（缓冲区溢出保护）
 *          2. 构造 tx_buf[32] —— 第一字节为地址 | 0x80，其余为 0x00（dummy）
 *          3. 一次 HAL_SPI_TransmitReceive() 完成全部传输
 *          4. 从 rx_buf[2] 开始复制 len 字节有效数据
 *
 *          为什么偏移 2 字节：
 *          - 第 0 字节：发送地址 (reg | 0x80)，同时接收 dummy（丢弃）
 *          - 第 1 字节：加速度计 SPI 协议要求的额外 dummy 周期
 *          - 第 2 字节起：实际数据
 *
 * @note    缓冲区限制为 30 字节 —— 加速度计突发最大需要 18 字节，远小于此限制
 */
void BMI088_accel_burst_read(uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (len > 30) return;

    /* 加速度计需要：1 字节地址 + 1 字节额外 dummy + 实际数据 */
    uint8_t tx_buf[32] = {0};
    uint8_t rx_buf[32] = {0};
    tx_buf[0] = reg | 0x80;    /*!< 设置读标志位 */

    BMI088_ACCEL_NS_L();
    HAL_SPI_TransmitReceive(&BMI088_USING_SPI_UNIT, tx_buf, rx_buf, len + 2, 100);
    BMI088_ACCEL_NS_H();

    for (uint8_t i = 0; i < len; i++)
    {
        buf[i] = rx_buf[i + 2];  /*!< 跳过地址字节和额外 dummy 字节 */
    }
}

/**
 * @brief  陀螺仪 SPI 突发读取（zzz 优化）
 *
 * @param  reg  起始寄存器地址
 * @param  buf  接收数据缓冲区
 * @param  len  有效数据字节数
 *
 * @details 与加速度计类似，但陀螺仪不需要额外的 dummy 周期：
 *          1. 安全检查：len > 31 则直接返回
 *          2. tx_buf[0] = reg | 0x80，其余为 0x00
 *          3. 总传输长度 = len + 1
 *          4. 有效数据从 rx_buf[1] 开始
 *
 *          为什么只偏移 1 字节：
 *          - 第 0 字节：发送地址，同时接收 dummy（丢弃）
 *          - 第 1 字节起：实际数据
 *
 * @note    缓冲区限制为 31 字节 —— 陀螺仪突发最大需要 6 字节，远小于此限制
 */
void BMI088_gyro_burst_read(uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (len > 31) return;

    /* 陀螺仪仅需：1 字节地址 + 实际数据（无额外 dummy） */
    uint8_t tx_buf[32] = {0};
    uint8_t rx_buf[32] = {0};
    tx_buf[0] = reg | 0x80;   /*!< 设置读标志位 */

    BMI088_GYRO_NS_L();
    HAL_SPI_TransmitReceive(&BMI088_USING_SPI_UNIT, tx_buf, rx_buf, len + 1, 100);
    BMI088_GYRO_NS_H();

    for(uint8_t i = 0; i < len; i++) {
        buf[i] = rx_buf[i + 1];  /*!< 跳过地址字节 */
    }
}
