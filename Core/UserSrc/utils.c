#include "utils.h"
#include "stm32h7xx_hal.h"
#include "arm_math.h"

/**
 * @brief 毫秒级延时 (阻塞 busy-wait)
 *
 * 不使用 HAL_Delay 的原因：HAL_Delay 内部依赖 SysTick 中断递增 uwTick，
 * 若在中断上下文或 SysTick 优先级较低时调用，可能被其他中断打断导致实际延时偏长。
 * 本实现通过循环 1000us 累积，在 busy-wait 中可精确等待。
 */
void DelayMs(uint32_t ms)
{
    while (ms--)
    {
        DelayUs(1000);   /* 逐次等待 1ms，避免 HAL_Delay 被中断干扰 */
    }
}

/**
 * @brief 微秒级延时 (阻塞 busy-wait)
 *
 * 记录起始微秒时间戳后反复查询，直到达到指定时长。
 * 由于使用 GetSysTimeUs 的 64 位时间戳，在微秒级延时中无需处理溢出。
 */
void DelayUs(uint32_t us)
{
    uint32_t t0 = GetSysTimeUs();
    while (GetSysTimeUs() - t0 < us)
        __NOP();
}

/**
 * @brief 检查 SysTick 计数器是否已溢出 (COUNTFLAG 置位)
 *
 * 当 SysTick 计数器从 1 递减到 0 且未重载时，CTRL 寄存器的 COUNTFLAG 位被硬件置 1。
 * 读取 CTRL 寄存器会自动清除该标志位。
 * 在 GetSysTimeUs 中用于检测是否需要重读 tick 和计数器值。
 */
__STATIC_INLINE uint32_t LL_SYSTICK_IsActiveCounterFlag()
{
    return ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == (SysTick_CTRL_COUNTFLAG_Msk));
}

/**
 * @brief 获取系统运行微秒时间戳 (64 位，无短期溢出)
 *
 * 竞态条件处理策略 (COUNTFLAG 双检查)：
 * 1. 先读取一次 CTRL 清掉旧 COUNTFLAG
 * 2. 读取 HAL tick 和 SysTick 当前计数值
 * 3. 再次检查 COUNTFLAG：若置位，说明步骤 2 期间发生了计数器溢出，
 *    此时读到的是溢出前的 VAL，需要重读 tick (已递增) 和 VAL (已重载)
 * 4. 最终时间 = m * 1000 + 当前计数值 * 1000 / 重载值
 *
 * 计算原理：
 * - SysTick 从 LOAD 递减到 0，递增值 = LOAD - VAL
 * - 经过的微秒数 = 递增值 / (LOAD+1) * 1000  (即经过 ticks / 总 ticks * 1000us)
 *
 * @return 64 位微秒时间戳，在 ~58.5 万年内不会溢出
 */
uint64_t GetSysTimeUs(void)
{
    /* 步骤1: 沉底旧 COUNTFLAG */
    LL_SYSTICK_IsActiveCounterFlag();
    /* 步骤2: 读取快照 */
    uint32_t m = HAL_GetTick();
    const uint32_t tms = SysTick->LOAD + 1;        /* 一个 tick 周期对应的计数器满量程值 */
    __IO uint32_t u = tms - SysTick->VAL;           /* 当前 tick 内已走过的计数值 */
    /* 步骤3: 检查竞态 —— 若读取期间发生溢出则重读 */
    if (LL_SYSTICK_IsActiveCounterFlag())
    {
        m = HAL_GetTick();                          /* 使用新的 (已递增) 毫秒值 */
        u = tms - SysTick->VAL;                     /* 使用新的 (已重载) 计数器值 */
    }
    /* 步骤4: 毫秒部分 * 1000 + 当前 tick 内的微秒插值 */
    return ((uint64_t)m * 1000 + ((uint64_t)u * 1000) / tms);
}

/**
 * @brief 获取系统运行毫秒时间戳 (32 位)
 *
 * 直接返回 HAL_GetTick()。注意 32 位在 ~49.7 天后回绕，
 * 若用于长时间段比较 (如 tone_start_ms_-now_ms)，
 * 需采用无符号减法技巧绕过回绕问题。
 */
uint32_t GetSysTimeMs(void)
{
    return HAL_GetTick();
}

float WrapPi(float angle_rad)
{
    while (angle_rad > _PI)
    {
        angle_rad -= _2PI;
    }
    while (angle_rad < -_PI)
    {
        angle_rad += _2PI;
    }
    return angle_rad;
}

/**
 * @brief 欧拉角 (弧度) 转四元数
 *
 * 旋转顺序：Z-Y-X (外旋), 即先绕 Z 轴转 yaw，再绕 Y 轴转 pitch，最后绕 X 轴转 roll。
 *
 * 使用半角公式：q = [cos(phi/2), sin(phi/2)*axis]
 * 组合三个欧拉旋转的四元数：q = q_z(yaw) * q_y(pitch) * q_x(roll)
 * 展开后得到解析公式：
 *   w = cr*cp*cy + sr*sp*sy
 *   x = sr*cp*cy - cr*sp*sy
 *   y = cr*sp*cy + sr*cp*sy
 *   z = cr*cp*sy - sr*sp*cy
 * 其中 cr=cos(roll/2), sr=sin(roll/2), cp=cos(pitch/2), sp=sin(pitch/2), cy=cos(yaw/2), sy=sin(yaw/2)
 *
 * @param roll_rad   横滚角 (弧度)
 * @param pitch_rad  俯仰角 (弧度)
 * @param yaw_rad    偏航角 (弧度)
 * @param q           输出四元数 [w, x, y, z]
 */
void EulerRad2Quaternion(float roll_rad, float pitch_rad, float yaw_rad, float *q)
{
    /* 计算各角度的半角三角函数 */
    float cr = arm_cos_f32(roll_rad * 0.5f);
    float sr = arm_sin_f32(roll_rad * 0.5f);
    float cp = arm_cos_f32(pitch_rad * 0.5f);
    float sp = arm_sin_f32(pitch_rad * 0.5f);
    float cy = arm_cos_f32(yaw_rad * 0.5f);
    float sy = arm_sin_f32(yaw_rad * 0.5f);

    /* Z-Y-X 欧拉四元数乘法展开式，顺序: qz * qy * qx */
    q[0] = cr * cp * cy + sr * sp * sy; /* w (实部) */
    q[1] = sr * cp * cy - cr * sp * sy; /* x (i 分量) */
    q[2] = cr * sp * cy + sr * cp * sy; /* y (j 分量) */
    q[3] = cr * cp * sy - sr * sp * cy; /* z (k 分量) */
}

/**
 * @brief 欧拉角 (度) 转四元数
 *
 * 内部转换为弧度后调用 EulerRad2Quaternion。
 */
void EulerDeg2Quaternion(float roll_deg, float pitch_deg, float yaw_deg, float *q)
{
    EulerRad2Quaternion(roll_deg * DEG_TO_RAD, pitch_deg * DEG_TO_RAD, yaw_deg * DEG_TO_RAD, q);
}

/**
 * @brief 四元数转欧拉角 (弧度)
 *
 * 从旋转矩阵与四元数对应关系中反推欧拉角。
 * 旋转矩阵 (Z-Y-X) 四元数形式：
 *   R[0][2] = 2*(q1*q3 - q0*q2) = -sin(pitch)
 *   R[0][0] = 1 - 2*(q1^2 + q2^2)  /  R[0][1] = 2*(q0*q3 + q1*q2)
 *
 * 推导：
 *   roll  = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1^2 + q2^2))
 *   pitch = asin(-2*(q1*q3 - q0*q2))
 *   yaw   = atan2(2*(q1*q2 + q0*q3), 1 - 2*(q2^2 + q3^2))
 *
 * 注意：pitch 超出 [-90°, +90°] 范围时会产生万向节死锁，atan2 结果可能跳变。
 *
 * @param q          输入四元数 [w, x, y, z]
 * @param roll_rad   输出横滚角 (弧度)
 * @param pitch_rad  输出俯仰角 (弧度)
 * @param yaw_rad    输出偏航角 (弧度)
 */
void Quaternion2EulerRad(float *q, float *roll_rad, float *pitch_rad, float *yaw_rad)
{
    *roll_rad  = atan2f(q[0] * q[1] + q[2] * q[3], 0.5f - q[1] * q[1] - q[2] * q[2]);
    const float sin_pitch = _constrain(-2.0f * (q[1] * q[3] - q[0] * q[2]), -1.0f, 1.0f);
    *pitch_rad = asinf(sin_pitch);
    *yaw_rad   = atan2f(q[1] * q[2] + q[0] * q[3], 0.5f - q[2] * q[2] - q[3] * q[3]);
}

/**
 * @brief 四元数转欧拉角 (度)
 *
 * 内部调用 Quaternion2EulerRad 获取弧度后乘以 RAD_TO_DEG 转为度。
 */
void Quaternion2EulerDeg(float *q, float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    float roll_rad, pitch_rad, yaw_rad;
    Quaternion2EulerRad(q, &roll_rad, &pitch_rad, &yaw_rad);
    *roll_deg = roll_rad * RAD_TO_DEG;
    *pitch_deg = pitch_rad * RAD_TO_DEG;
    *yaw_deg = yaw_rad * RAD_TO_DEG;
}
