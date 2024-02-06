#ifndef __BSP_TLE5012_H__
#define __BSP_TLE5012_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "std_config.h"

void Tle5012_DMA_read(void);
void Init_tle5012(void);
uint16_t encoder_read(void);

#ifdef __cplusplus
}
#endif

#endif // __BSP_TLE5012_H__
