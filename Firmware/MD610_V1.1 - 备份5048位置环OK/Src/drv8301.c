#include "drv8301.h"

/***********************HAL库****************************/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_tim.h"
#include "parameters_conversion.h"
#include "stm32f4xx.h"



void DRV8301_Init(void)
{
	
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOB_CLK_ENABLE();           //开启GPIOB时钟
	
    GPIO_Initure.Pin = DRV8301_EN_GATE_Pin; //PF9,10
    GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;  //推挽输出
    GPIO_Initure.Pull = GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed = GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init( DRV8301_EN_GATE_PORT , &GPIO_Initure );
	
		DRV8301_EN_GATE_LOW;
	
}


