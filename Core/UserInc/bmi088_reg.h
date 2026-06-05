/**
 * @file    bmi088_reg.h
 * @brief   BMI088 6 轴 IMU 完整寄存器映射 —— 地址定义与位域编码
 *
 * @details 
 *
 * @note    
 */

#ifndef BMI088REG_H
#define BMI088REG_H

/*===========================================================================
 * 加速度计 (Accelerometer) 寄存器映射
 *===========================================================================*/

/* 加速度计芯片 ID 寄存器 (0x00) —— 读回固定值 0x1E 表示加速度计正常 */
#define BMI088_ACC_CHIP_ID 0x00
#define BMI088_ACC_CHIP_ID_VALUE 0x1E

/* 加速度计错误寄存器 (0x02) —— 指示致命错误和配置错误 */
#define BMI088_ACC_ERR_REG 0x02
#define BMI088_ACCEL_CONGIF_ERROR_SHFITS 0x2    /*!< 配置错误标志位偏移 */
#define BMI088_ACCEL_CONGIF_ERROR (1 << BMI088_ACCEL_CONGIF_ERROR_SHFITS)
#define BMI088_FATAL_ERROR_SHFITS 0x0           /*!< 致命错误标志位偏移 */
#define BMI088_FATAL_ERROR (1 << BMI088_FATAL_ERROR)

/* 加速度计状态寄存器 (0x03) —— 包含 DRDY 数据就绪标志 */
#define BMI088_ACC_STATUS 0x03
#define BMI088_ACCEL_DRDY_SHFITS 0x7            /*!< 加速度数据就绪标志位偏移 */
#define BMI088_ACCEL_DRDY (1 << BMI088_ACCEL_DRDY_SHFITS)

/* 加速度计数据输出寄存器 (0x12-0x17) —— 三轴加速度原始值 (LSB/MSB 各 1 字节) */
#define BMI088_ACCEL_XOUT_L 0x12    /*!< X 轴加速度低字节 */
#define BMI088_ACCEL_XOUT_M 0x13    /*!< X 轴加速度高字节 */
#define BMI088_ACCEL_YOUT_L 0x14    /*!< Y 轴加速度低字节 */
#define BMI088_ACCEL_YOUT_M 0x15    /*!< Y 轴加速度高字节 */
#define BMI088_ACCEL_ZOUT_L 0x16    /*!< Z 轴加速度低字节 */
#define BMI088_ACCEL_ZOUT_M 0x17    /*!< Z 轴加速度高字节 */

/* 传感器内部时间戳寄存器 (0x18-0x1A) —— 3 字节传感器运行时间 */
#define BMI088_SENSORTIME_DATA_L 0x18
#define BMI088_SENSORTIME_DATA_M 0x19
#define BMI088_SENSORTIME_DATA_H 0x1A

/* 加速度计中断状态寄存器 1 (0x1D) —— DRDY 中断状态 */
#define BMI088_ACC_INT_STAT_1 0x1D
#define BMI088_ACCEL_DRDY_INTERRUPT_SHFITS 0x7
#define BMI088_ACCEL_DRDY_INTERRUPT (1 << BMI088_ACCEL_DRDY_INTERRUPT_SHFITS)

/* 加速度计温度数据寄存器 (0x22-0x23) —— 11 位有符号温度值 */
#define BMI088_TEMP_M 0x22    /*!< 温度高字节（包含 bit 10-3） */
#define BMI088_TEMP_L 0x23    /*!< 温度低字节（bit 2-0 在高 3 位） */

/** 加速度计配置寄存器 (0x40) —— 带宽和 ODR 选择
 *  @note bit 7 必须置 1 (BMI088_ACC_CONF_MUST_Set)
 */
#define BMI088_ACC_CONF 0x40
#define BMI088_ACC_CONF_MUST_Set 0x80                     /*!< 必须位：写入时必须置 1 */
#define BMI088_ACC_BWP_SHFITS 0x4                         /*!< 滤波器带宽位偏移 */
#define BMI088_ACC_OSR4 (0x0 << BMI088_ACC_BWP_SHFITS)    /*!< 过采样率 4x */
#define BMI088_ACC_OSR2 (0x1 << BMI088_ACC_BWP_SHFITS)    /*!< 过采样率 2x */
#define BMI088_ACC_NORMAL (0x2 << BMI088_ACC_BWP_SHFITS)   /*!< 正常带宽模式 */

#define BMI088_ACC_ODR_SHFITS 0x0                         /*!< 输出数据速率位偏移 */
#define BMI088_ACC_12_5_HZ (0x5 << BMI088_ACC_ODR_SHFITS) /*!< ODR 12.5 Hz */
#define BMI088_ACC_25_HZ (0x6 << BMI088_ACC_ODR_SHFITS)   /*!< ODR 25 Hz */
#define BMI088_ACC_50_HZ (0x7 << BMI088_ACC_ODR_SHFITS)   /*!< ODR 50 Hz */
#define BMI088_ACC_100_HZ (0x8 << BMI088_ACC_ODR_SHFITS)  /*!< ODR 100 Hz */
#define BMI088_ACC_200_HZ (0x9 << BMI088_ACC_ODR_SHFITS)  /*!< ODR 200 Hz */
#define BMI088_ACC_400_HZ (0xA << BMI088_ACC_ODR_SHFITS)  /*!< ODR 400 Hz */
#define BMI088_ACC_800_HZ (0xB << BMI088_ACC_ODR_SHFITS)  /*!< ODR 800 Hz */
#define BMI088_ACC_1600_HZ (0xC << BMI088_ACC_ODR_SHFITS) /*!< ODR 1600 Hz */

/* 加速度计量程选择寄存器 (0x41) */
#define BMI088_ACC_RANGE 0x41
#define BMI088_ACC_RANGE_SHFITS 0x0
#define BMI088_ACC_RANGE_3G (0x0 << BMI088_ACC_RANGE_SHFITS)   /*!< 量程 +/- 3g */
#define BMI088_ACC_RANGE_6G (0x1 << BMI088_ACC_RANGE_SHFITS)   /*!< 量程 +/- 6g */
#define BMI088_ACC_RANGE_12G (0x2 << BMI088_ACC_RANGE_SHFITS)  /*!< 量程 +/- 12g */
#define BMI088_ACC_RANGE_24G (0x3 << BMI088_ACC_RANGE_SHFITS)  /*!< 量程 +/- 24g */

/* INT1 IO 控制寄存器 (0x53) —— INT1 引脚输出模式配置 */
#define BMI088_INT1_IO_CTRL 0x53
#define BMI088_ACC_INT1_IO_ENABLE_SHFITS 0x3
#define BMI088_ACC_INT1_IO_ENABLE (0x1 << BMI088_ACC_INT1_IO_ENABLE_SHFITS) /*!< 使能 INT1 输出 */
#define BMI088_ACC_INT1_GPIO_MODE_SHFITS 0x2
#define BMI088_ACC_INT1_GPIO_PP (0x0 << BMI088_ACC_INT1_GPIO_MODE_SHFITS)  /*!< 推挽输出 */
#define BMI088_ACC_INT1_GPIO_OD (0x1 << BMI088_ACC_INT1_GPIO_MODE_SHFITS)  /*!< 开漏输出 */
#define BMI088_ACC_INT1_GPIO_LVL_SHFITS 0x1
#define BMI088_ACC_INT1_GPIO_LOW (0x0 << BMI088_ACC_INT1_GPIO_LVL_SHFITS)  /*!< 低电平有效 */
#define BMI088_ACC_INT1_GPIO_HIGH (0x1 << BMI088_ACC_INT1_GPIO_LVL_SHFITS) /*!< 高电平有效 */

/* INT2 IO 控制寄存器 (0x54) —— INT2 引脚输出模式配置 */
#define BMI088_INT2_IO_CTRL 0x54
#define BMI088_ACC_INT2_IO_ENABLE_SHFITS 0x3
#define BMI088_ACC_INT2_IO_ENABLE (0x1 << BMI088_ACC_INT2_IO_ENABLE_SHFITS)
#define BMI088_ACC_INT2_GPIO_MODE_SHFITS 0x2
#define BMI088_ACC_INT2_GPIO_PP (0x0 << BMI088_ACC_INT2_GPIO_MODE_SHFITS)
#define BMI088_ACC_INT2_GPIO_OD (0x1 << BMI088_ACC_INT2_GPIO_MODE_SHFITS)
#define BMI088_ACC_INT2_GPIO_LVL_SHFITS 0x1
#define BMI088_ACC_INT2_GPIO_LOW (0x0 << BMI088_ACC_INT2_GPIO_LVL_SHFITS)
#define BMI088_ACC_INT2_GPIO_HIGH (0x1 << BMI088_ACC_INT2_GPIO_LVL_SHFITS)

/* 中断映射数据寄存器 (0x58) —— 选择中断源映射到 INT1/INT2 引脚 */
#define BMI088_INT_MAP_DATA 0x58
#define BMI088_ACC_INT2_DRDY_INTERRUPT_SHFITS 0x6
#define BMI088_ACC_INT2_DRDY_INTERRUPT (0x1 << BMI088_ACC_INT2_DRDY_INTERRUPT_SHFITS) /*!< DRDY 映射到 INT2 */
#define BMI088_ACC_INT1_DRDY_INTERRUPT_SHFITS 0x2
#define BMI088_ACC_INT1_DRDY_INTERRUPT (0x1 << BMI088_ACC_INT1_DRDY_INTERRUPT_SHFITS) /*!< DRDY 映射到 INT1 */

/* 加速度计自检寄存器 (0x6D) */
#define BMI088_ACC_SELF_TEST 0x6D
#define BMI088_ACC_SELF_TEST_OFF 0x00
#define BMI088_ACC_SELF_TEST_POSITIVE_SIGNAL 0x0D     /*!< 正向自检激励信号 */
#define BMI088_ACC_SELF_TEST_NEGATIVE_SIGNAL 0x09     /*!< 负向自检激励信号 */

/* 加速度计电源配置寄存器 (0x7C) */
#define BMI088_ACC_PWR_CONF 0x7C
#define BMI088_ACC_PWR_SUSPEND_MODE 0x03              /*!< 挂起模式（低功耗） */
#define BMI088_ACC_PWR_ACTIVE_MODE 0x00               /*!< 激活模式（正常工作） */

/* 加速度计电源控制寄存器 (0x7D) */
#define BMI088_ACC_PWR_CTRL 0x7D
#define BMI088_ACC_ENABLE_ACC_OFF 0x00                /*!< 禁用加速度计 */
#define BMI088_ACC_ENABLE_ACC_ON 0x04                 /*!< 使能加速度计 */

/* 加速度计软复位寄存器 (0x7E) —— 写入 0xB6 触发软复位 */
#define BMI088_ACC_SOFTRESET 0x7E
#define BMI088_ACC_SOFTRESET_VALUE 0xB6

/*===========================================================================
 * 陀螺仪 (Gyroscope) 寄存器映射
 *===========================================================================*/

/* 陀螺仪芯片 ID 寄存器 (0x00) —— 读回固定值 0x0F 表示陀螺仪正常 */
#define BMI088_GYRO_CHIP_ID 0x00
#define BMI088_GYRO_CHIP_ID_VALUE 0x0F

/* 陀螺仪数据输出寄存器 (0x02-0x07) —— 三轴角速度原始值 (X/Y/Z LSB/MSB) */
#define BMI088_GYRO_X_L 0x02    /*!< X 轴角速度低字节 */
#define BMI088_GYRO_X_H 0x03    /*!< X 轴角速度高字节 */
#define BMI088_GYRO_Y_L 0x04    /*!< Y 轴角速度低字节 */
#define BMI088_GYRO_Y_H 0x05    /*!< Y 轴角速度高字节 */
#define BMI088_GYRO_Z_L 0x06    /*!< Z 轴角速度低字节 */
#define BMI088_GYRO_Z_H 0x07    /*!< Z 轴角速度高字节 */

/* 陀螺仪中断状态寄存器 1 (0x0A) —— DRDY 状态标志 */
#define BMI088_GYRO_INT_STAT_1 0x0A
#define BMI088_GYRO_DYDR_SHFITS 0x7
#define BMI088_GYRO_DYDR (0x1 << BMI088_GYRO_DYDR_SHFITS)

/* 陀螺仪量程寄存器 (0x0F) */
#define BMI088_GYRO_RANGE 0x0F
#define BMI088_GYRO_RANGE_SHFITS 0x0
#define BMI088_GYRO_2000 (0x0 << BMI088_GYRO_RANGE_SHFITS)   /*!< 量程 +/- 2000 dps */
#define BMI088_GYRO_1000 (0x1 << BMI088_GYRO_RANGE_SHFITS)   /*!< 量程 +/- 1000 dps */
#define BMI088_GYRO_500 (0x2 << BMI088_GYRO_RANGE_SHFITS)    /*!< 量程 +/- 500 dps */
#define BMI088_GYRO_250 (0x3 << BMI088_GYRO_RANGE_SHFITS)    /*!< 量程 +/- 250 dps */
#define BMI088_GYRO_125 (0x4 << BMI088_GYRO_RANGE_SHFITS)    /*!< 量程 +/- 125 dps */

/** 陀螺仪带宽寄存器 (0x10) —— ODR 和滤波器截止频率
 *  @note bit 7 必须置 1 (BMI088_GYRO_BANDWIDTH_MUST_Set)
 *  @note 第一个数字表示 ODR，第二个数字表示 3dB 带宽
 */
#define BMI088_GYRO_BANDWIDTH 0x10
#define BMI088_GYRO_BANDWIDTH_MUST_Set 0x80    /*!< 必须位：写入时必须置 1 */
#define BMI088_GYRO_2000_532_HZ 0x00           /*!< ODR 2000 Hz, 带宽 532 Hz */
#define BMI088_GYRO_2000_230_HZ 0x01           /*!< ODR 2000 Hz, 带宽 230 Hz */
#define BMI088_GYRO_1000_116_HZ 0x02           /*!< ODR 1000 Hz, 带宽 116 Hz */
#define BMI088_GYRO_400_47_HZ 0x03             /*!< ODR 400 Hz,  带宽 47 Hz */
#define BMI088_GYRO_200_23_HZ 0x04             /*!< ODR 200 Hz,  带宽 23 Hz */
#define BMI088_GYRO_100_12_HZ 0x05             /*!< ODR 100 Hz,  带宽 12 Hz */
#define BMI088_GYRO_200_64_HZ 0x06             /*!< ODR 200 Hz,  带宽 64 Hz */
#define BMI088_GYRO_100_32_HZ 0x07             /*!< ODR 100 Hz,  带宽 32 Hz */

/* 陀螺仪低功耗模式 1 寄存器 (0x11) */
#define BMI088_GYRO_LPM1 0x11
#define BMI088_GYRO_NORMAL_MODE 0x00            /*!< 正常模式 */
#define BMI088_GYRO_SUSPEND_MODE 0x80           /*!< 挂起模式 */
#define BMI088_GYRO_DEEP_SUSPEND_MODE 0x20      /*!< 深度挂起模式 */

/* 陀螺仪软复位寄存器 (0x14) —— 写入 0xB6 触发软复位 */
#define BMI088_GYRO_SOFTRESET 0x14
#define BMI088_GYRO_SOFTRESET_VALUE 0xB6

/* 陀螺仪控制寄存器 (0x15) —— DRDY 中断使能/禁用 */
#define BMI088_GYRO_CTRL 0x15
#define BMI088_DRDY_OFF 0x00    /*!< 禁用数据就绪中断 */
#define BMI088_DRDY_ON 0x80     /*!< 使能数据就绪中断 */

/* INT3/INT4 IO 配置寄存器 (0x16) */
#define BMI088_GYRO_INT3_INT4_IO_CONF 0x16
#define BMI088_GYRO_INT4_GPIO_MODE_SHFITS 0x3
#define BMI088_GYRO_INT4_GPIO_PP (0x0 << BMI088_GYRO_INT4_GPIO_MODE_SHFITS)  /*!< INT4 推挽输出 */
#define BMI088_GYRO_INT4_GPIO_OD (0x1 << BMI088_GYRO_INT4_GPIO_MODE_SHFITS)  /*!< INT4 开漏输出 */
#define BMI088_GYRO_INT4_GPIO_LVL_SHFITS 0x2
#define BMI088_GYRO_INT4_GPIO_LOW (0x0 << BMI088_GYRO_INT4_GPIO_LVL_SHFITS)  /*!< INT4 低电平有效 */
#define BMI088_GYRO_INT4_GPIO_HIGH (0x1 << BMI088_GYRO_INT4_GPIO_LVL_SHFITS) /*!< INT4 高电平有效 */
#define BMI088_GYRO_INT3_GPIO_MODE_SHFITS 0x1
#define BMI088_GYRO_INT3_GPIO_PP (0x0 << BMI088_GYRO_INT3_GPIO_MODE_SHFITS)  /*!< INT3 推挽输出 */
#define BMI088_GYRO_INT3_GPIO_OD (0x1 << BMI088_GYRO_INT3_GPIO_MODE_SHFITS)  /*!< INT3 开漏输出 */
#define BMI088_GYRO_INT3_GPIO_LVL_SHFITS 0x0
#define BMI088_GYRO_INT3_GPIO_LOW (0x0 << BMI088_GYRO_INT3_GPIO_LVL_SHFITS)  /*!< INT3 低电平有效 */
#define BMI088_GYRO_INT3_GPIO_HIGH (0x1 << BMI088_GYRO_INT3_GPIO_LVL_SHFITS) /*!< INT3 高电平有效 */

/* INT3/INT4 中断映射寄存器 (0x18) —— 选择中断源映射到 INT3/INT4 */
#define BMI088_GYRO_INT3_INT4_IO_MAP 0x18
#define BMI088_GYRO_DRDY_IO_OFF 0x00                                              /*!< DRDY 不映射到任何中断引脚 */
#define BMI088_GYRO_DRDY_IO_INT3 0x01                                             /*!< DRDY 映射到 INT3 */
#define BMI088_GYRO_DRDY_IO_INT4 0x80                                             /*!< DRDY 映射到 INT4 */
#define BMI088_GYRO_DRDY_IO_BOTH (BMI088_GYRO_DRDY_IO_INT3 | BMI088_GYRO_DRDY_IO_INT4) /*!< DRDY 同时映射到 INT3 和 INT4 */

/* 陀螺仪自检寄存器 (0x3C) —— BIST (Built-In Self-Test) 控制和状态 */
#define BMI088_GYRO_SELF_TEST 0x3C
#define BMI088_GYRO_RATE_OK_SHFITS 0x4
#define BMI088_GYRO_RATE_OK (0x1 << BMI088_GYRO_RATE_OK_SHFITS)          /*!< 速率自检通过 */
#define BMI088_GYRO_BIST_FAIL_SHFITS 0x2
#define BMI088_GYRO_BIST_FAIL (0x1 << BMI088_GYRO_BIST_FAIL_SHFITS)     /*!< BIST 失败标志 */
#define BMI088_GYRO_BIST_RDY_SHFITS 0x1
#define BMI088_GYRO_BIST_RDY (0x1 << BMI088_GYRO_BIST_RDY_SHFITS)       /*!< BIST 就绪标志 */
#define BMI088_GYRO_TRIG_BIST_SHFITS 0x0
#define BMI088_GYRO_TRIG_BIST (0x1 << BMI088_GYRO_TRIG_BIST_SHFITS)     /*!< 触发 BIST */

#endif
