#ifndef __LED_H
#define __LED_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "std_config.h"

#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_tim.h"
#include "parameters_conversion.h"

/***************LED GPIO¶¨Òå******************/
#define  LED_PORT          GPIOB
#define  GREEN_LED_Pin     GPIO_PIN_0

#define  LED_PORT_RED       GPIOB
#define  RED_LED_Pin        GPIO_PIN_1

#define  LED_GREEN_ON       HAL_GPIO_WritePin(LED_PORT,GREEN_LED_Pin,GPIO_PIN_RESET)	
#define  LED_GREEN_OFF      HAL_GPIO_WritePin(LED_PORT,GREEN_LED_Pin,GPIO_PIN_SET)	
#define  LED_GREEN_TOGGLE   LED_PORT->ODR ^= GREEN_LED_Pin

#define  LED_RED_ON      	  HAL_GPIO_WritePin(LED_PORT,RED_LED_Pin,GPIO_PIN_RESET)	
#define  LED_RED_OFF        HAL_GPIO_WritePin(LED_PORT,RED_LED_Pin,GPIO_PIN_SET)	
#define  LED_RED_TOGGLE     LED_PORT_RED->ODR ^= RED_LED_Pin

    
    
void LED_Init(void);




#ifdef __cplusplus
}
#endif

#endif /* __LED_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
