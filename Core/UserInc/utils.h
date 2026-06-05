/**
 * @file    utils.h
 * @brief   通用工具函数与宏定义
 */
#ifndef UTILS_H
#define UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define _PI     (3.14159265359f)
#define _PI_2   (1.57079632679f)
#define _2PI    (6.28318530718f)

/* 转速转角速度：RPM -> rad/s, 公式: RPM * 2*PI / 60 */
#define RPM_TO_RADPS          (0.10471975f)
/* 角速度转转速：rad/s -> RPM, 公式: rad/s * 60 / (2*PI) */
#define RADPS_TO_RPM          (9.54929659f)
/* 角度转弧度：deg -> rad, 公式: deg * PI / 180 */
#define DEG_TO_RAD          (0.01745329252f)
/* 弧度转角度：rad -> deg, 公式: rad * 180 / PI */
#define RAD_TO_DEG          (57.2957795131f)

/* 值限幅：将 amt 限制在 [low, high] 闭区间内 */
#define _constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
/* 取两者中的较大值 */
#define _max(a,b)           ((a) > (b) ? (a) : (b))
/* 取两者中的较小值 */
#define _min(a,b)           ((a) < (b) ? (a) : (b))

#define DMA_UNION_BUF_SIZE_BYTES         (512U)                         /*!< DMA 缓冲区总字节数 */
#define DMA_UNION_BUF_SIZE_FLOATS        (DMA_UNION_BUF_SIZE_BYTES/4)   /*!< DMA 缓冲区可容纳的 float 个数 (128) */
/**
 * @brief DMA 联合体缓冲区
 *
 * 通过 union 将同一块 512 字节内存映射为 4 种类型的视图，
 * 实现无拷贝的类型切换。典型用途：
 * - f_data: VOFA JustFloat 浮点数据填充
 * - c_data: snprintf 格式化字符串输出
 * - u8_data: 原始字节 DMA 发送
 * - u32_data: 32 位对齐访问
 */
union DmaUnionBuffer {
    float f_data[DMA_UNION_BUF_SIZE_FLOATS];       /*!< 浮点视图：128 个 float */
    uint32_t u32_data[DMA_UNION_BUF_SIZE_FLOATS];  /*!< 32位视图：128 个 uint32_t */
    char c_data[DMA_UNION_BUF_SIZE_BYTES];         /*!< 字符视图：512 个 char */
    uint8_t u8_data[DMA_UNION_BUF_SIZE_BYTES];     /*!< 字节视图：512 个 uint8_t */
};

/**
 * @brief 微秒级延时 (阻塞)
 *
 * 通过反复读取 SysTick 计数器实现 busy-wait 微秒延时。
 * 适用于短延时 (几微秒到数毫秒)，长延时建议用 DelayMs 配合调度器。
 */
void DelayUs(uint32_t us);

/**
 * @brief 毫秒级延时 (阻塞)
 *
 * 内部循环调用 DelayUs(1000)，避免使用 HAL_Delay 被其他中断干扰问题。
 */
void DelayMs(uint32_t ms);

/**
 * @brief 获取系统运行微秒时间戳 (64 位)
 * @return 自启动以来的微秒数
 */
uint64_t GetSysTimeUs(void);

/**
 * @brief 获取系统运行毫秒时间戳 (32 位)
 *
 * 直接返回 HAL_GetTick() 值。注意 32 位约 49.7 天后回绕。
 *
 * @return 自启动以来的毫秒数
 */
uint32_t GetSysTimeMs(void);

/**
 * @brief 欧拉角 (弧度) 转四元数
 *
 * 采用标准 Z-Y-X (yaw-pitch-roll) 旋转顺序：
 * 先绕 z 转 yaw，再绕 y 转 pitch，最后绕 x 转 roll。
 * 使用半角公式：q = [cos(phi/2), sin(phi/2)*axis]
 * 输出四元数：q[0]=w, q[1]=x, q[2]=y, q[3]=z
 *
 * @param roll_rad   横滚角 (弧度)
 * @param pitch_rad  俯仰角 (弧度)
 * @param yaw_rad    偏航角 (弧度)
 * @param q           输出四元数数组 (长度 4)
 */
void EulerRad2Quaternion(float roll_rad, float pitch_rad, float yaw_rad, float *q);

/**
 * @brief 欧拉角 (度) 转四元数
 *
 * 内部转弧度后调用 EulerRad2Quaternion。
 */
void EulerDeg2Quaternion(float roll_deg, float pitch_deg, float yaw_deg, float *q);

/**
 * @brief 四元数转欧拉角 (弧度)
 *
 * 从四元数反算 roll(横滚)、pitch(俯仰)、yaw(偏航) 弧度值。
 *
 * 推导公式：
 * - roll  = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1^2 + q2^2))
 * - pitch = asin(-2*(q1*q3 - q0*q2))
 * - yaw   = atan2(2*(q1*q2 + q0*q3), 1 - 2*(q2^2 + q3^2))
 *
 * @param q          输入四元数数组 (w,x,y,z)
 * @param roll_rad   输出横滚角 (弧度)
 * @param pitch_rad  输出俯仰角 (弧度)
 * @param yaw_rad    输出偏航角 (弧度)
 */
void Quaternion2EulerRad(float *q, float *roll_rad, float *pitch_rad, float *yaw_rad);

/**
 * @brief 四元数转欧拉角 (度)
 *
 * 内部调用 Quaternion2EulerRad 后转度。
 */
void Quaternion2EulerDeg(float *q, float *roll_deg, float *pitch_deg, float *yaw_deg);

#ifdef __cplusplus
}
#endif

#endif