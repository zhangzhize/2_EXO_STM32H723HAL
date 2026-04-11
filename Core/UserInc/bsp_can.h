#ifndef BSP_CAN_H
#define BSP_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "fdcan.h"

void BspCanInit(void);
void FDCanSendData(uint32_t id, uint32_t id_type, uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* BSP_CAN_H */