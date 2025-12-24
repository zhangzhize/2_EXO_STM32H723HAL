#ifndef UTILS_H
#define UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define _PI     (3.14159265359f)
#define _PI_2   (1.57079632679f)
#define _2PI    (6.28318530718f)
#define RPM_TO_RAD          (0.10471975f)
#define RAD_TO_RPM          (9.54929659f)
#define DEG_TO_RAD          (0.01745329252f)
#define RAD_TO_DEG          (57.2957795131f)
#define _constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define _max(a,b)           ((a) > (b) ? (a) : (b))
#define _min(a,b)           ((a) < (b) ? (a) : (b))

void DelayUs(uint32_t us);
void DelayMs(uint32_t ms);
uint64_t GetSysTimeUs(void);
uint32_t GetSysTimeMs(void);

#ifdef __cplusplus
}
#endif

#endif