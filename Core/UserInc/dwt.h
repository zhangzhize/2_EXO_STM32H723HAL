#ifndef DWT_H
#define DWT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

#define  DWT_CYCCNT  *(volatile unsigned int *)0xE0001004
#define  DWT_CR      *(volatile unsigned int *)0xE0001000
#define  DEM_CR      *(volatile unsigned int *)0xE000EDFC
#define  DBGMCU_CR   *(volatile unsigned int *)0xE0042004

void DWTInit(void);
void DWTDelayUs(uint32_t us);
void DWTDelayTicks(uint32_t delay_ticks);
uint64_t DWTGetSysTimeUs64(void);
uint32_t DWTGetDeltaUs(uint32_t start_ticks);


#ifdef __cplusplus
}
#endif

#endif  // DWT_H
