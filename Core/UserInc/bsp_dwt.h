/**
 * @file    bsp_dwt.h
 * @brief   BSP DWT 周期计数器 —— 外骨骼高精度定时/延时硬件抽象层
 *
 * @details 使用 Cortex-M7 内核的 DWT (Data Watchpoint and Trace) 单元中的
 *          CYCCNT 寄存器实现微秒级精确定时。相比 SysTick，DWT CYCCNT 提供：
 *          - 直接读取 CPU 周期计数，无需中断开销
 *          - 更高的分辨率（1 个 CPU 周期，约 1.82 ns @ 550 MHz）
 *          - 微秒延时和节拍延时两种模式
 *
 *          64 位单调时间戳：
 *          由于 CYCCNT 是 32 位寄存器，约 7.8 秒会回绕一次（@ 550 MHz）。
 *          DWTGetSysTimeUs64() 通过维护软件高 32 位解决回绕问题，提供
 *          单调递增的 64 位微秒时间戳，适合长时间运行的外骨骼系统。
 *
 *          外骨骼应用场景：
 *          - 电机控制环路中的精确延时
 *          - 传感器采样间隔精确控制
 *          - 状态机超时判断
 *          - 性能分析与调优
 *
 * @note    DEM_CR_TRCENA 必须在 DWT_CR_CYCCNTENA 之前使能，
 *          否则 DWT 寄存器不可写。DBGMCU_CR 在调试模式下保持计数器运行。
 */

#ifndef BSP_DWT_H
#define BSP_DWT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

/* DWT 周期计数寄存器地址 (0xE0001004) —— 读取当前 CPU 周期数 */
#define  DWT_CYCCNT  *(volatile unsigned int *)0xE0001004
/* DWT 控制寄存器地址 (0xE0001000) —— 使能 CYCCNT */
#define  DWT_CR      *(volatile unsigned int *)0xE0001000
/* 调试异常及监控控制寄存器地址 (0xE000EDFC) —— TRCENA 位使能 DWT */
#define  DEM_CR      *(volatile unsigned int *)0xE000EDFC
/* 调试 MCU 配置寄存器地址 (0xE0042004) —— 调试模式下保持定时器运行 */
#define  DBGMCU_CR   *(volatile unsigned int *)0xE0042004

/* 初始化 DWT 周期计数器：使能 TRCENA -> 清零 CYCCNT -> 使能 CYCCNTENA */
void DWTInit(void);
/* 微秒级延时 —— 基于 CPU 周期计数的精确延时 */
void DWTDelayUs(uint32_t us);
/* 节拍级延时 —— 直接指定延时周期数 */
void DWTDelayTicks(uint32_t delay_ticks);
/* 获取 64 位单调递增微秒时间戳 —— 通过软件扩展解决 32 位 CYCCNT 回绕问题 */
uint64_t DWTGetSysTimeUs64(void);
/* 从给定起始节拍计算经过的微秒数 —— 利用无符号减法自动处理 32 位回绕 */
uint32_t DWTGetDeltaUs(uint32_t start_ticks);


#ifdef __cplusplus
}
#endif

#endif  // BSP_DWT_H
