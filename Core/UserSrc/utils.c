#include "utils.h"
#include "stm32h7xx_hal.h"

void DelayMs(uint32_t ms)
{
    HAL_Delay(ms);
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

