#include "bsp_dwt.h"
#include "stm32h7xx_hal.h"

/* DEM_CR 寄存器 TRCENA 位掩码 —— 使能 DWT 和 ITM 单元 */
#define  DEM_CR_TRCENA               (1 << 24)
/* DWT_CR 寄存器 CYCCNTENA 位掩码 —— 使能周期计数器 */
#define  DWT_CR_CYCCNTENA            (1 <<  0)

/* 每微秒对应的 CPU 周期数，由 DWTInit() 根据系统时钟频率计算 */
static uint32_t g_TicksPerUs = 0;

/**
 * @brief  初始化 DWT 周期计数器
 *
 * @details 初始化顺序必须严格遵循：
 *          TRCENA -> 清零 CYCCNT -> CYCCNTENA。
 *          如果 TRCENA 未置位，后续对 DWT 寄存器的写操作将被忽略。
 *          清零 CYCCNT 确保计时从已知起点开始。
 */
void DWTInit(void)
{
    DEM_CR         |= (unsigned int)DEM_CR_TRCENA;      /*!< 使能 DWT 单元调试访问 */
    DWT_CYCCNT      = (unsigned int)0u;                  /*!< 清零周期计数器 */
    DWT_CR         |= (unsigned int)DWT_CR_CYCCNTENA;    /*!< 启动周期计数器 */

    /* 根据系统时钟频率计算每微秒对应的 CPU 周期数 */
    g_TicksPerUs = HAL_RCC_GetSysClockFreq() / 1000000U;
}

/**
 * @brief  微秒级精确延时
 * @param  _ulDelayTime  延时的微秒数
 *
 * @details 将微秒数转换为 CPU 周期数后进行节拍延时。
 *          循环中计算实际经过的周期数，利用无符号整数减法的
 *          模运算特性自动处理计数器 32 位回绕。
 *
 * @note    SystemCoreClock 为全局变量，由 HAL 在 SystemClock_Config() 中设置。
 *          对于 H723 系列，SystemCoreClock 典型值为 550 MHz。
 */
void DWTDelayUs(uint32_t _ulDelayTime)
{
    uint32_t tCnt, tDelayCnt;
    uint32_t tStart;

    tStart = DWT_CYCCNT;     /*!< 刚进入函数时的计数器值（起始参考点） */
    tCnt = 0;
    tDelayCnt = _ulDelayTime * (SystemCoreClock / 1000000); /*!< 需要的 CPU 周期总数 */

    while(tCnt < tDelayCnt)
    {
        /* 无符号减法自动处理 32 位回绕：即使 CYCCNT 回绕到 0，差值仍然正确 */
        tCnt = DWT_CYCCNT - tStart;
    }
}

/**
 * @brief  指定节拍数的精确延时
 * @param  delay_ticks  延时的 CPU 周期数
 *
 * @details 与 DWTDelayUs() 类似，但不进行微秒到周期的转换，
 *          直接以给定的周期数作为延时目标。
 *
 * @note   当发生第一次 32 位计数器回绕时，无符号减法仍然可以正确计算差值。
 */
void DWTDelayTicks(uint32_t delay_ticks)
{
    uint32_t tCnt, tDelayCnt;
    uint32_t tStart;

    tCnt = 0;
    tDelayCnt = delay_ticks;       /*!< 需要的节拍数 */
    tStart = DWT_CYCCNT;           /*!< 刚进入函数时的计数器值 */

    while(tCnt < tDelayCnt)
    {
        /* 无符号减法自动处理 32 位回绕 */
        tCnt = DWT_CYCCNT - tStart;
    }
}


/**
 * @brief  获取单调递增的 64 位系统时间（微秒）
 *
 * @details 解决 32 位 CYCCNT 回绕的方案：
 *          - s_last_cyccnt：记录上一次的 CYCCNT 值
 *          - s_high_word：软件维护的高 32 位计数器
 *          - 每次调用时，若当前 CYCCNT < 上一次值，说明发生了 32 位回绕，
 *            s_high_word 自增
 *          - 最终时间戳 = (s_high_word << 32 | CYCCNT) / g_TicksPerUs
 *
 *          对于 550 MHz CPU，回绕周期约 7.8 秒，64 位扩展后可覆盖数千年。
 *
 * @warning 两次调用间隔不得超过一次 CYCCNT 回绕周期（~7.8s @ 550MHz），
 *          否则可能遗漏回绕事件。对于外骨骼的 1 kHz 控制循环（1ms 间隔），
 *          这完全不是问题。
 *
 * @return  64 位无符号微秒时间戳，若 g_TicksPerUs==0（未初始化）则返回 0
 */
uint64_t DWTGetSysTimeUs64(void)
{
    static uint32_t s_last_cyccnt = 0;  /*!< 上一次读取的 CYCCNT 值，用于回绕检测 */
    static uint32_t s_high_word = 0;    /*!< 软件维护的高 32 位，CYCCNT 每回绕一次自增 */

    uint32_t cur = DWT_CYCCNT;
    /* 检测 32 位回绕：当前值小于上一次记录值说明计数器已回绕 */
    if (cur < s_last_cyccnt)
    {
        s_high_word++;
    }
    s_last_cyccnt = cur;

    /* 组合 64 位周期计数 */
    uint64_t ticks64 = ( (uint64_t)s_high_word << 32 ) | (uint64_t)cur;

    if (g_TicksPerUs == 0)
    {
        return 0;  /*!< 未初始化，返回 0 防止除零 */
    }
    return ticks64 / (uint64_t)g_TicksPerUs;
}



/**
 * @brief  计算从指定起始节拍到当前时刻经过的微秒数
 * @param  start_ticks  起始时刻的 DWT_CYCCNT 值
 * @return 经过的微秒数，若 g_TicksPerUs==0 则返回 0
 *
 * @details 利用无符号整数减法的模运算特性，即使 CYCCNT 在
 *          start_ticks 之后发生了回绕，delta_ticks 仍然正确。
 *
 *          典型用法：记录操作开始时的 CYCCNT，结束时调用此函数
 *          获取操作耗时。适用于超时判断和性能测量。
 */
uint32_t DWTGetDeltaUs(uint32_t start_ticks)
{
    uint32_t delta_ticks = DWT_CYCCNT - start_ticks;  /*!< 无符号减法自动处理回绕 */
    if (g_TicksPerUs == 0)
    {
        return 0;
    }
    return delta_ticks / g_TicksPerUs;
}
