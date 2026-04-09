#include "dwt.h"
#include "stm32h7xx_hal.h"

#define  DEM_CR_TRCENA               (1 << 24)
#define  DWT_CR_CYCCNTENA            (1 <<  0)

static uint32_t g_TicksPerUs = 0;

void DWTInit(void)
{
	DEM_CR         |= (unsigned int)DEM_CR_TRCENA;   
	DWT_CYCCNT      = (unsigned int)0u;
	DWT_CR         |= (unsigned int)DWT_CR_CYCCNTENA;

    /* 计算每个时钟周期的微秒数 */
    g_TicksPerUs = HAL_RCC_GetSysClockFreq() / 1000000U;
}

void DWTDelayUs(uint32_t _ulDelayTime)
{
    uint32_t tCnt, tDelayCnt;
	uint32_t tStart;
		
	tStart = DWT_CYCCNT;                                     /* 刚进入时的计数器值 */
	tCnt = 0;
	tDelayCnt = _ulDelayTime * (SystemCoreClock / 1000000);	 /* 需要的循环次数 */

	while(tCnt < tDelayCnt)
	{
		tCnt = DWT_CYCCNT - tStart; /* 计算实际经过的周期数，如果超过32位无符号整数范围，则需要处理溢出问题 */
	}
}

void DWTDelayTicks(uint32_t delay_ticks)
{
    uint32_t tCnt, tDelayCnt;
	uint32_t tStart;
		
	tCnt = 0;
	tDelayCnt = delay_ticks;	 /* 需要的节拍数 */ 		      
	tStart = DWT_CYCCNT;         /* 刚进入时的计数器值 */
	
	while(tCnt < tDelayCnt)
	{
		tCnt = DWT_CYCCNT - tStart; /* 求减过程中，如果发生第一次32位计数器重新计数，依然可以正确计算 */	
	}
}


/**
 * @brief  获取单调递增的系统时间（微秒，64位）
 * @note   由 DWT_CYCCNT 扩展为 64-bit，解决 32-bit CYCCNT 回绕问题。
 *         要求：DWTInit() 已启用 CYCCNT；g_TicksPerUs 已正确初始化且非 0。
 */
uint64_t DWTGetSysTimeUs64(void)
{
    static uint32_t s_last_cyccnt = 0;
    static uint32_t s_high_word = 0;

    uint32_t cur = DWT_CYCCNT;
    if (cur < s_last_cyccnt)
    {
        s_high_word++;
    }
    s_last_cyccnt = cur;

    uint64_t ticks64 = ( (uint64_t)s_high_word << 32 ) | (uint64_t)cur;

    if (g_TicksPerUs == 0)
    {
        return 0;
    }
    return ticks64 / (uint64_t)g_TicksPerUs;
}



uint32_t DWTGetDeltaUs(uint32_t start_ticks)
{
    uint32_t delta_ticks = DWT_CYCCNT - start_ticks;
    if (g_TicksPerUs == 0)
    {
        return 0;
    }
    return delta_ticks / g_TicksPerUs;
}




