#ifndef __DRV8301_H
#define __DRV8301_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"



#define  DRV8301_EN_GATE_PORT          		GPIOB
#define  DRV8301_EN_GATE_Pin     			 		GPIO_PIN_5

#define  DRV8301_EN_GATE_LOW       	 			HAL_GPIO_WritePin(DRV8301_EN_GATE_PORT,DRV8301_EN_GATE_Pin,GPIO_PIN_RESET)
#define  DRV8301_EN_GATE_HIGH      				HAL_GPIO_WritePin(DRV8301_EN_GATE_PORT,DRV8301_EN_GATE_Pin,GPIO_PIN_SET)
#define  DRV8301_EN_GATE_TOGGLE    				DRV8301_EN_GATE_PORT->ODR ^= DRV8301_EN_GATE_Pin

#define  DRV8301_FAULT								 		

void DRV8301_Init(void);


#ifdef __cplusplus
}
#endif

#endif /* __DRV8301_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
