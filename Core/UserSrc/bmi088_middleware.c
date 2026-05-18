#include "bmi088_middleware.h"
#include "main.h"
#include "utils.h"

#define BMI088_USING_SPI_UNIT   hspi2

extern SPI_HandleTypeDef BMI088_USING_SPI_UNIT;

/**
************************************************************************
* @brief:      	BMI088_GPIO_init(void)
* @param:       void
* @retval:     	void
* @details:    	BMI088传感器GPIO初始化函数
************************************************************************
**/
void BMI088_GPIO_init(void)
{

}
/**
************************************************************************
* @brief:      	BMI088_com_init(void)
* @param:       void
* @retval:     	void
* @details:    	BMI088传感器通信初始化函数
************************************************************************
**/
void BMI088_com_init(void)
{


}
/**
************************************************************************
* @brief:      	BMI088_delay_ms(uint16_t ms)
* @param:       ms - 要延迟的毫秒数
* @retval:     	void
* @details:    	延迟指定毫秒数的函数，基于微秒延迟实现
************************************************************************
**/
void BMI088_delay_ms(uint16_t ms)
{
    //zzz
    DelayMs(ms);
    return;
    
    while(ms--)
    {
        BMI088_delay_us(1000);
    }
}
/**
************************************************************************
* @brief:      	BMI088_delay_us(uint16_t us)
* @param:       us - 要延迟的微秒数
* @retval:     	void
* @details:    	微秒级延迟函数，使用SysTick定时器实现
************************************************************************
**/
void BMI088_delay_us(uint16_t us)
{
    //zzz
    DelayUs(us);
    return;
}
/**
************************************************************************
* @brief:      	BMI088_ACCEL_NS_L(void)
* @param:       void
* @retval:     	void
* @details:    	将BMI088加速度计片选信号置低，使其处于选中状态
************************************************************************
**/
void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(BMI088_ACC_CS_GPIO_Port, BMI088_ACC_CS_Pin, GPIO_PIN_RESET);
}
/**
************************************************************************
* @brief:      	BMI088_ACCEL_NS_H(void)
* @param:       void
* @retval:     	void
* @details:    	将BMI088加速度计片选信号置高，使其处于非选中状态
************************************************************************
**/
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(BMI088_ACC_CS_GPIO_Port, BMI088_ACC_CS_Pin, GPIO_PIN_SET);
}
/**
************************************************************************
* @brief:      	BMI088_GYRO_NS_L(void)
* @param:       void
* @retval:     	void
* @details:    	将BMI088陀螺仪片选信号置低，使其处于选中状态
************************************************************************
**/
void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_RESET);
}
/**
************************************************************************
* @brief:      	BMI088_GYRO_NS_H(void)
* @param:       void
* @retval:     	void
* @details:    	将BMI088陀螺仪片选信号置高，使其处于非选中状态
************************************************************************
**/
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_SET);
}
/**
************************************************************************
* @brief:      	BMI088_read_write_byte(uint8_t txdata)
* @param:       txdata - 要发送的数据
* @retval:     	uint8_t - 接收到的数据
* @details:    	通过BMI088使用的SPI总线进行单字节的读写操作
************************************************************************
**/
uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&BMI088_USING_SPI_UNIT, &txdata, &rx_data, 1, 1000);
    return rx_data;
}

/**
 * @brief added by zzz
 * 
 * @param reg 
 * @param buf 
 * @param len 
 */
void BMI088_accel_burst_read(uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (len > 30) return;

    // 加速度计需要：1字节地址 + 1字节Dummy + 真实数据
    uint8_t tx_buf[32] = {0};
    uint8_t rx_buf[32] = {0};
    tx_buf[0] = reg | 0x80;

    BMI088_ACCEL_NS_L();
    HAL_SPI_TransmitReceive(&BMI088_USING_SPI_UNIT, tx_buf, rx_buf, len + 2, 100);
    BMI088_ACCEL_NS_H();

    for (uint8_t i = 0; i < len; i++)
    {
        buf[i] = rx_buf[i + 2];
    }
}

/**
 * @brief added by zzz
 * 
 * @param reg 
 * @param buf 
 * @param len 
 */
void BMI088_gyro_burst_read(uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (len > 31) return;

    // 陀螺仪需要：1字节地址 + 真实数据
    uint8_t tx_buf[32] = {0};
    uint8_t rx_buf[32] = {0};
    tx_buf[0] = reg | 0x80;
    
    BMI088_GYRO_NS_L();
    HAL_SPI_TransmitReceive(&BMI088_USING_SPI_UNIT, tx_buf, rx_buf, len + 1, 100);
    BMI088_GYRO_NS_H();

    for(uint8_t i = 0; i < len; i++) {
        buf[i] = rx_buf[i + 1];
    }
}

