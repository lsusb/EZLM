#ifndef __HQFOC_H__
#define __HQFOC_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "std_config.h"
#include "control.h"

#define POLE_PITCH 12.5       // 磁极距离，单位mm
#define ENC_COUNT_PER_MM 2048 // 每mm多少CNT
#define ENC_COUNT_PER_POLE_PAIR (int32_t)(25 * ENC_COUNT_PER_MM)
#define EANGLE_MANGLE_RATE 65536 / ENC_COUNT_PER_POLE_PAIR

typedef enum
{
	POS_FRAME, // 位置数据帧
	MAG_FRAME  // 磁场强度数据帧
} ams5311_frame_type_e;

void drv_init(void);
uint8_t drv_fault_check(void);
uint16_t HQ_FOC_CurrControllerM1(void);
float get_Vbus_voltage(void);
void set_pid(void);


#ifdef __cplusplus
}
#endif

#endif // __HQFOC_H__
