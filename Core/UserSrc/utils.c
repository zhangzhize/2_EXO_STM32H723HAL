#include "utils.h"
#include "stm32h7xx_hal.h"
#include "arm_math.h"

void DelayMs(uint32_t ms)
{
    // HAL_Delay(ms);  /**< 注意: HAL_Delay()函数内部使用了SysTick中断, 在某些情况下可能会被其他中断打断, 导致实际延时时间超过预期; 实际延时 ms+1 */
    while (ms--)
    {
        DelayUs(1000);
    }
}

void DelayUs(uint32_t us)
{
    uint32_t t0 = GetSysTimeUs();
    while (GetSysTimeUs() - t0 < us)
        __NOP();
}

__STATIC_INLINE uint32_t LL_SYSTICK_IsActiveCounterFlag()
{
    return ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == (SysTick_CTRL_COUNTFLAG_Msk));
}

uint64_t GetSysTimeUs(void)
{
    LL_SYSTICK_IsActiveCounterFlag();
    uint32_t m = HAL_GetTick();
    const uint32_t tms = SysTick->LOAD + 1;
    __IO uint32_t u = tms - SysTick->VAL;
    if (LL_SYSTICK_IsActiveCounterFlag())
    {
        m = HAL_GetTick();
        u = tms - SysTick->VAL;
    }
    return (m * 1000 + (u * 1000) / tms);
}

uint32_t GetSysTimeMs(void)
{
    return HAL_GetTick();
}


void EulerRad2Quaternion(float roll_rad, float pitch_rad, float yaw_rad, float *q)
{
    float cr = arm_cos_f32(roll_rad * 0.5f);
    float sr = arm_sin_f32(roll_rad * 0.5f);
    float cp = arm_cos_f32(pitch_rad * 0.5f);
    float sp = arm_sin_f32(pitch_rad * 0.5f);
    float cy = arm_cos_f32(yaw_rad * 0.5f);
    float sy = arm_sin_f32(yaw_rad * 0.5f);

    q[0] = cr * cp * cy + sr * sp * sy; // w (Real)
    q[1] = sr * cp * cy - cr * sp * sy; // x (i)
    q[2] = cr * sp * cy + sr * cp * sy; // y (j)
    q[3] = cr * cp * sy - sr * sp * cy; // z (k)
}

void EulerDeg2Quaternion(float roll_deg, float pitch_deg, float yaw_deg, float *q)
{
    EulerRad2Quaternion(roll_deg * DEG_TO_RAD, pitch_deg * DEG_TO_RAD, yaw_deg * DEG_TO_RAD, q);
}

void Quaternion2EulerRad(float *q, float *roll_rad, float *pitch_rad, float *yaw_rad)
{
    *roll_rad  = atan2f(q[0] * q[1] + q[2] * q[3], 0.5f - q[1] * q[1] - q[2] * q[2]);
    *pitch_rad = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2]));
    *yaw_rad   = atan2f(q[1] * q[2] + q[0] * q[3], 0.5f - q[2] * q[2] - q[3] * q[3]);
}

void Quaternion2EulerDeg(float *q, float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    float roll_rad, pitch_rad, yaw_rad;
    Quaternion2EulerRad(q, &roll_rad, &pitch_rad, &yaw_rad);
    *roll_deg = roll_rad * RAD_TO_DEG;
    *pitch_deg = pitch_rad * RAD_TO_DEG;
    *yaw_deg = yaw_rad * RAD_TO_DEG;
}
