/**
 * @file    bmi088_driver.h
 * @brief   BMI088 6 轴 IMU 传感器驱动 —— 外骨骼姿态感知核心
 * 
 * @details 
 * @note    灵敏度宏根据选用量程自动匹配，修改量程时需同步修改灵敏度值。
 *          当前配置：加速度计 3g，陀螺仪 2000 dps。
 */

#ifndef BMI088DRIVER_H
#define BMI088DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

/* 温度转换系数：每 LSB 对应 0.125 摄氏度 */
#define BMI088_TEMP_FACTOR 0.125f
/* 温度转换偏移：0 摄氏度对应的 ADC 输出 */
#define BMI088_TEMP_OFFSET 23.0f

/* 加速度计初始化时需要写入的配置寄存器数量 */
#define BMI088_WRITE_ACCEL_REG_NUM 6
/* 陀螺仪初始化时需要写入的配置寄存器数量 */
#define BMI088_WRITE_GYRO_REG_NUM 6

/* 数据就绪状态位掩码 */
#define BMI088_GYRO_DATA_READY_BIT 0          /*!< 陀螺仪数据就绪 */
#define BMI088_ACCEL_DATA_READY_BIT 1         /*!< 加速度计数据就绪 */
#define BMI088_ACCEL_TEMP_DATA_READY_BIT 2    /*!< 加速度计温度数据就绪 */

/* 长时间延时 (ms) —— 用于传感器软复位后的稳定等待 */
#define BMI088_LONG_DELAY_TIME 80
/* 通信等待时间 (us) —— SPI 读写之间的最短间隔 */
#define BMI088_COM_WAIT_SENSOR_TIME 150

/* 加速度计 I2C 从机地址（7 位地址左移 1 位）—— SPI 模式下不使用 */
#define BMI088_ACCEL_IIC_ADDRESSE (0x18 << 1)
/* 陀螺仪 I2C 从机地址（7 位地址左移 1 位）—— SPI 模式下不使用 */
#define BMI088_GYRO_IIC_ADDRESSE (0x68 << 1)

/* 加速度计量程选择 —— 当前使用 3g */
#define BMI088_ACCEL_RANGE_3G
//#define BMI088_ACCEL_RANGE_6G
//#define BMI088_ACCEL_RANGE_12G
//#define BMI088_ACCEL_RANGE_24G

/* 陀螺仪量程选择 —— 当前使用 2000 dps */
#define BMI088_GYRO_RANGE_2000
//#define BMI088_GYRO_RANGE_1000
//#define BMI088_GYRO_RANGE_500
//#define BMI088_GYRO_RANGE_250
//#define BMI088_GYRO_RANGE_125

/* 加速度计灵敏度 (g/LSB) —— 不同量程对应不同的转换系数 */
#define BMI088_ACCEL_3G_SEN 0.0008974358974f
#define BMI088_ACCEL_6G_SEN 0.00179443359375f
#define BMI088_ACCEL_12G_SEN 0.0035888671875f
#define BMI088_ACCEL_24G_SEN 0.007177734375f

/* 陀螺仪灵敏度 (dps/LSB) —— 不同量程对应不同的转换系数 */
#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f
#define BMI088_GYRO_1000_SEN 0.00053263221801584764920766930190693f
#define BMI088_GYRO_500_SEN 0.00026631610900792382460383465095346f
#define BMI088_GYRO_250_SEN 0.00013315805450396191230191732547673f
#define BMI088_GYRO_125_SEN 0.000066579027251980956150958662738366f

/**
 * @brief BMI088 原始数据结构体（带 __attribute__((packed)) 对齐）
 *
 * @details 用于 DMA 突发读取时的数据解析。包含：
 *          - status：数据状态字节
 *          - accel[3]：三轴加速度原始值 (int16_t)
 *          - temp：温度原始值 (int16_t)
 *          - gyro[3]：三轴陀螺仪原始值 (int16_t)
 */
typedef struct BMI088_RAW_DATA
{
    uint8_t status;
    int16_t accel[3];
    int16_t temp;
    int16_t gyro[3];
}__attribute__((packed)) bmi088_raw_data_t;

/**
 * @brief BMI088 物理量数据结构体
 *
 * @details 将原始 ADC 值乘以灵敏度系数后得到的实际物理量：
 *          - accel[3]：加速度 (g)
 *          - temp：温度 (摄氏度)
 *          - gyro[3]：角速度 (dps)
 *          - time：时间戳（可关联 DWT 时间）
 */
typedef struct BMI088_REAL_DATA
{
    uint8_t status;
    float accel[3];
    float temp;
    float gyro[3];
    float time;
} bmi088_real_data_t;

/* BMI088 初始化错误码 —— 逐位累积，支持多错误同时报告 */
enum
{
    BMI088_NO_ERROR = 0x00,                 /*!< 无错误 */
    BMI088_ACC_PWR_CTRL_ERROR = 0x01,       /*!< 加速度计电源控制寄存器写入失败 */
    BMI088_ACC_PWR_CONF_ERROR = 0x02,       /*!< 加速度计电源配置寄存器写入失败 */
    BMI088_ACC_CONF_ERROR = 0x03,           /*!< 加速度计配置寄存器写入失败 */
    BMI088_ACC_SELF_TEST_ERROR = 0x04,      /*!< 加速度计自检失败 */
    BMI088_ACC_RANGE_ERROR = 0x05,          /*!< 加速度计量程设置写入失败 */
    BMI088_INT1_IO_CTRL_ERROR = 0x06,       /*!< INT1 引脚 IO 配置写入失败 */
    BMI088_INT_MAP_DATA_ERROR = 0x07,       /*!< 中断映射寄存器写入失败 */
    BMI088_GYRO_RANGE_ERROR = 0x08,         /*!< 陀螺仪量程设置写入失败 */
    BMI088_GYRO_BANDWIDTH_ERROR = 0x09,     /*!< 陀螺仪带宽设置写入失败 */
    BMI088_GYRO_LPM1_ERROR = 0x0A,          /*!< 陀螺仪低功耗模式设置写入失败 */
    BMI088_GYRO_CTRL_ERROR = 0x0B,          /*!< 陀螺仪控制寄存器写入失败 */
    BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,   /*!< INT3/INT4 IO 配置写入失败 */
    BMI088_GYRO_INT3_INT4_IO_MAP_ERROR = 0x0D,    /*!< INT3/INT4 中断映射写入失败 */

    BMI088_SELF_TEST_ACCEL_ERROR = 0x80,    /*!< 加速度计硬件自检失败 */
    BMI088_SELF_TEST_GYRO_ERROR = 0x40,     /*!< 陀螺仪硬件自检失败 */
    BMI088_NO_SENSOR = 0xFF,                /*!< 未检测到传感器（芯片 ID 不匹配） */
};

extern uint8_t BMI088_init(void);
extern uint8_t bmi088_accel_init(void);
extern uint8_t bmi088_gyro_init(void);

/**
 * @brief  读取 BMI088 全部传感器数据（加速度 + 陀螺仪 + 温度）
 * @param  gyro[3]     输出：陀螺仪三轴角速度 (dps)
 * @param  accel[3]    输出：加速度计三轴加速度 (g)
 * @param  temperature 输出：温度值 (摄氏度)
 *
 * @details 内部使用 SPI 突发读取（burst read）一次读取加速度计的
 *          18 字节数据（加速度 + 温度 + 传感器时间）和陀螺仪的
 *          6 字节数据（角速度），然后进行灵敏度转换。
 */
extern void BMI088_read(float gyro[3], float accel[3], float *temperate);

#ifdef __cplusplus
}
#endif

#endif
