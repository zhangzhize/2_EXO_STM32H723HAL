#include "bmi088_driver.h"
#include "bmi088_reg.h"
#include "bmi088_middleware.h"

/* 加速度计当前使用的灵敏度系数 —— 根据量程选择宏确定 */
float BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
/* 陀螺仪当前使用的灵敏度系数 —— 根据量程选择宏确定 */
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;


#if defined(BMI088_USE_SPI)

/**
 * @brief  加速度计单寄存器写入宏
 *
 * @details 执行流程：
 *          1. CS 拉低选中加速度计
 *          2. 发送寄存器地址（bit 7 = 0 表示写操作）
 *          3. 发送数据字节
 *          4. CS 拉高释放
 */
#define BMI088_accel_write_single_reg(reg, data) \
    {                                            \
        BMI088_ACCEL_NS_L();                     \
        BMI088_write_single_reg((reg), (data));  \
        BMI088_ACCEL_NS_H();                     \
    }

/**
 * @brief  加速度计单寄存器读取宏
 *
 * @details 协议要求：
 *          1. 发送 (reg | 0x80) —— bit 7 = 1 表示读操作
 *          2. 发送 0x55 作为 dummy 字节，同时接收实际数据
 *          3. 再发送 0x55，接收到的数据即为寄存器值
 *          （第二个 0x55 是因为加速度计 SPI 协议需要额外的时钟周期）
 */
#define BMI088_accel_read_single_reg(reg, data) \
    {                                           \
        BMI088_ACCEL_NS_L();                    \
        BMI088_read_write_byte((reg) | 0x80);   \
        BMI088_read_write_byte(0x55);           \
        (data) = BMI088_read_write_byte(0x55);  \
        BMI088_ACCEL_NS_H();                    \
    }

/**
 * @brief  加速度计连续多寄存器读取宏
 *
 * @details 先发送起始地址（| 0x80），然后调用 BMI088_read_muli_reg()
 *          从该地址开始连续读取 len 字节。
 */
#define BMI088_accel_read_muli_reg(reg, data, len) \
    {                                              \
        BMI088_ACCEL_NS_L();                       \
        BMI088_read_write_byte((reg) | 0x80);      \
        BMI088_read_muli_reg(reg, data, len);      \
        BMI088_ACCEL_NS_H();                       \
    }

/**
 * @brief  陀螺仪单寄存器写入宏 —— 与加速度计类似但使用陀螺仪的片选信号
 */
#define BMI088_gyro_write_single_reg(reg, data) \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_write_single_reg((reg), (data)); \
        BMI088_GYRO_NS_H();                     \
    }

/**
 * @brief  陀螺仪单寄存器读取宏 —— 陀螺仪读协议比加速度计简单（无额外 dummy）
 */
#define BMI088_gyro_read_single_reg(reg, data)  \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_read_single_reg((reg), &(data)); \
        BMI088_GYRO_NS_H();                     \
    }

/**
 * @brief  陀螺仪连续多寄存器读取宏

#define BMI088_gyro_read_muli_reg(reg, data, len)   \
    {                                               \
        BMI088_GYRO_NS_L();                         \
        BMI088_read_muli_reg((reg), (data), (len)); \
        BMI088_GYRO_NS_H();                         \
    }
 */

/* 内部 SPI 原语函数声明 */
static void BMI088_write_single_reg(uint8_t reg, uint8_t data);
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data);
//static void BMI088_write_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len );
// static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

#elif defined(BMI088_USE_IIC)


#endif

/**
 * @brief  加速度计初始化配置表
 *
 * @details 每个条目包含三个值：{寄存器地址, 写入值, 校验失败时的错误码}
 *
 *          配置序列解释：
 *          [0] ACC_PWR_CTRL   = 0x04 —— 使能加速度计（上电）
 *          [1] ACC_PWR_CONF   = 0x00 —— 激活模式（非挂起）
 *          [2] ACC_CONF       = 0xA0 —— 正常带宽 + 800Hz ODR + 必设位
 *          [3] ACC_RANGE      = 0x00 —— 量程 3g
 *          [4] INT1_IO_CTRL   = 0x09 —— INT1 输出使能 + 推挽输出 + 低电平有效
 *          [5] INT_MAP_DATA   = 0x04 —— INT1 映射到 DRDY（数据就绪）中断
 */
static uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] =
    {
        {BMI088_ACC_PWR_CTRL,   BMI088_ACC_ENABLE_ACC_ON,         BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF,   BMI088_ACC_PWR_ACTIVE_MODE,       BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF,       BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE,      BMI088_ACC_RANGE_3G,              BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL,   BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA,   BMI088_ACC_INT1_DRDY_INTERRUPT,   BMI088_INT_MAP_DATA_ERROR}

};

/**
 * @brief  陀螺仪初始化配置表
 *
 * @details 配置序列解释：
 *          [0] GYRO_RANGE             = 0x00 —— 量程 2000 dps
 *          [1] GYRO_BANDWIDTH         = 0x82 —— ODR 1000Hz + 带宽 116Hz + 必设位
 *          [2] GYRO_LPM1              = 0x00 —— 正常模式（非低功耗）
 *          [3] GYRO_CTRL              = 0x80 —— 使能 DRDY 中断
 *          [4] INT3_INT4_IO_CONF      = 0x01 —— INT3 推挽输出 + 低电平有效
 *          [5] INT3_INT4_IO_MAP      = 0x01 —— INT3 映射到陀螺仪 DRDY
 */
static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] =
    {
        {BMI088_GYRO_RANGE,            BMI088_GYRO_2000,                                                       BMI088_GYRO_RANGE_ERROR},
        {BMI088_GYRO_BANDWIDTH,        BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set,                BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1,             BMI088_GYRO_NORMAL_MODE,                                                 BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL,             BMI088_DRDY_ON,                                                          BMI088_GYRO_CTRL_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_CONF,BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW,                    BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3,                                               BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}

};

/**
 * @brief  BMI088 整体初始化
 *
 * @details 执行顺序：
 *          1. GPIO 初始化（片选引脚）
 *          2. SPI 通信接口初始化
 *          3. 加速度计初始化（通信检查 + 复位 + 芯片 ID + 配置寄存器写入）
 *          4. 陀螺仪初始化（通信检查 + 复位 + 芯片 ID + 配置寄存器写入）
 *
 * @return 错误码（0 表示成功，非 0 是各阶段错误的 OR 组合）
 */
uint8_t BMI088_init(void)
{
    uint8_t error = BMI088_NO_ERROR;
    /* GPIO 和 SPI 初始化由 CubeMX 生成，此处调用中间件完成平台相关设置 */
    BMI088_GPIO_init();
    BMI088_com_init();

    error |= bmi088_accel_init();
    error |= bmi088_gyro_init();

    return error;
}

/**
 * @brief  加速度计初始化
 *
 * @details 完整初始化序列（遵循 BMI088 数据手册建议）：
 *
 *          Step 1: 通信检查 —— 读芯片 ID 寄存器两次，确认 SPI 通信正常
 *          Step 2: 软复位 —— 写入 SOFTRESET 命令 (0xB6)，等待 80ms 稳定
 *          Step 3: 复位后通信检查 —— 再次读芯片 ID 确认传感器已就绪
 *          Step 4: 芯片 ID 校验 —— 检查返回值是否为 0x1E
 *          Step 5: 循环写配置序列 —— 逐个寄存器写入并回读校验
 *
 *          写后回读校验：每写入一个配置寄存器后，立即读取验证。
 *          这种方案能可靠检测 SPI 通信错误或传感器未就绪状态。
 *
 * @return BMI088_NO_ERROR (0) 或对应的错误码
 */
uint8_t bmi088_accel_init(void)
{
    uint8_t res = 0;
    uint8_t write_reg_num = 0;

    /* Step 1: 通信检查 —— 连续两次读取芯片 ID */
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    /* Step 2: 软复位 */
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    /* Step 3: 复位后通信检查 */
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    /* Step 4: 芯片 ID 校验 —— "Who Am I" 应为 0x1E */
    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    /* Step 5: 配置序列写入和回读校验 */
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {
        BMI088_accel_write_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], write_BMI088_accel_reg_data_error[write_reg_num][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_accel_read_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_accel_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_accel_reg_data_error[write_reg_num][2];
        }
    }
    return BMI088_NO_ERROR;
}

/**
 * @brief  陀螺仪初始化
 *
 * @details 初始化流程与加速度计相同（通信检查 -> 复位 -> ID 校验 -> 配置序列）。
 *          BMI088 的陀螺仪和加速度计是独立芯片，需要分别初始化和验证。
 *
 *          陀螺仪芯片 ID 值为 0x0F。
 *
 * @return BMI088_NO_ERROR 或对应错误码
 */
uint8_t bmi088_gyro_init(void)
{
    uint8_t write_reg_num = 0;
    uint8_t res = 0;

    /* Step 1: 通信检查 */
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    /* Step 2: 软复位 */
    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    /* Step 3: 复位后通信检查 */
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    /* Step 4: 芯片 ID 校验 —— "Who Am I" 应为 0x0F */
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    /* Step 5: 配置序列写入和回读校验 */
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {
        BMI088_gyro_write_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], write_BMI088_gyro_reg_data_error[write_reg_num][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_gyro_read_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_gyro_reg_data_error[write_reg_num][2];
        }
    }

    return BMI088_NO_ERROR;
}

/**
 * @brief  批量读取 BMI088 全部传感器数据
 *
 * @param  gyro[3]     输出：陀螺仪 X/Y/Z 角速度 (dps)
 * @param  accel[3]    输出：加速度计 X/Y/Z 加速度 (g)
 * @param  temperate   输出：温度 (摄氏度)
 *
 * @details 数据解析说明：
 *
 *          加速度计数据（18 字节突发读取，起始地址 0x12）：
 *          - buf[0:1] = ACCEL_X (LSB, MSB)
 *          - buf[2:3] = ACCEL_Y (LSB, MSB)
 *          - buf[4:5] = ACCEL_Z (LSB, MSB)
 *          - buf[6:8] = SENSORTIME (传感器内部时间戳，未使用)
 *          - buf[9:14] = 状态和额外数据（未使用）
 *          - buf[16:17] = 温度数据（11 位有效值）
 *
 *          陀螺仪数据（6 字节突发读取，起始地址 0x02）：
 *          - buf[0:1] = GYRO_X (LSB, MSB)
 *          - buf[2:3] = GYRO_Y (LSB, MSB)
 *          - buf[4:5] = GYRO_Z (LSB, MSB)
 *
 *          温度解析：11 位有符号数，范围为 -1024 到 +1023，
 *          转换公式：T(C) = value * 0.125 + 23.0
 *
 *          zzz 修改说明：
 *          原始代码使用逐寄存器多字节读取 (read_muli_reg)，
 *          改为使用底层的 burst_read 函数实现一次性 SPI 传输，
 *          减少片选切换次数，提高读取效率。
 */
void BMI088_read(float gyro[3], float accel[3], float *temperate)
{
    // uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
    uint8_t buf[18] = {0};
    int16_t bmi088_raw_temp;

    /* zzz: 使用底层突发读取 API —— 一次 SPI 传输获取 18 字节加速度计数据 */
    // BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);
    BMI088_accel_burst_read(BMI088_ACCEL_XOUT_L, buf, 18);

    /* 解析加速度计数据：每 2 字节组成 int16_t，乘以灵敏度系数 */
    bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;

    /* 解析温度：11 位有符号数（buf[16] 的低 8 位 + buf[17] 的高 3 位） */
    bmi088_raw_temp = (int16_t)((buf[16] << 3) | (buf[17] >> 5));
    if (bmi088_raw_temp > 1023) {
        bmi088_raw_temp -= 2048;   /*!< 将 11 位无符号值转换为有符号 */
    }
    *temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

    /* zzz: 使用底层突发读取 API —— 一次 SPI 传输获取 6 字节陀螺仪数据 */
    // BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
    // if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    // {
    //     bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    //     gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
    //     bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    //     gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
    //     bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
    //     gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
    // }
    BMI088_gyro_burst_read(BMI088_GYRO_X_L, buf, 6);

    /* 解析陀螺仪数据：每 2 字节组成 int16_t，乘以灵敏度系数 */
    bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
    bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;

    /* zzz 废弃：原温度读取代码 —— 现在温度从加速度计 18 字节突发中获取 */
    // BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);
    // bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
    // if (bmi088_raw_temp > 1023)
    // {
    //     bmi088_raw_temp -= 2048;
    // }
    // *temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

#if defined(BMI088_USE_SPI)

/**
 * @brief  SPI 单寄存器写入原语
 * @param  reg   寄存器地址（bit 7 = 0，写操作）
 * @param  data  要写入的数据
 *
 * @details 先发送寄存器地址字节，再发送数据字节。
 *          片选信号由调用宏（accel/gyro_write_single_reg）控制。
 */
static void BMI088_write_single_reg(uint8_t reg, uint8_t data)
{
    BMI088_read_write_byte(reg);
    BMI088_read_write_byte(data);
}

/**
 * @brief  SPI 单寄存器读取原语
 * @param  reg          寄存器地址（bit 7 由调用宏置 1）
 * @param  return_data  输出：读取到的寄存器值
 *
 * @details 发送 (reg | 0x80) 表示读操作，然后发送 0x55 作为 dummy 字节
 *          同时接收实际数据。0x55 是任意的，只要不干扰传感器即可。
 */
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data)
{
    BMI088_read_write_byte(reg | 0x80);
    *return_data = BMI088_read_write_byte(0x55);
}

//static void BMI088_write_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len )
//{
//    BMI088_read_write_byte( reg );
//    while( len != 0 )
//    {
//
//        BMI088_read_write_byte( *buf );
//        buf ++;
//        len --;
//    }
//
//}

/**
 * @brief  SPI 连续多寄存器读取原语
 * @param  reg  起始寄存器地址（bit 7 由调用宏置 1）
 * @param  buf  接收数据缓冲区
 * @param  len  要读取的字节数
 *
 * @details 先发送地址，然后循环发送 0x55（dummy）并接收数据。
 *          因为 SPI 是全双工协议，要接收数据必须发送等量的 dummy 字节。
 *          采用 0x55 作为 dummy 值——0x55 = 0b01010101，能满足大多数外设的
 *          SPI 时序要求，且不触发任何特殊命令。
 */
// static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
// {
//     BMI088_read_write_byte(reg | 0x80);

//     while (len != 0)
//     {
//         *buf = BMI088_read_write_byte(0x55);
//         buf++;
//         len--;
//     }
// }

#elif defined(BMI088_USE_IIC)

#endif
