#include "drv8301.h"

/***********************HAL��****************************/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_tim.h"
#include "parameters_conversion.h"
#include "stm32f4xx.h"



void DRV8301_Init(void)
{
	
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOB_CLK_ENABLE();           //����GPIOBʱ��
	
    GPIO_Initure.Pin = DRV8301_EN_GATE_Pin; //PF9,10
    GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;  //�������
    GPIO_Initure.Pull = GPIO_PULLUP;          //����
    GPIO_Initure.Speed = GPIO_SPEED_HIGH;     //����
    HAL_GPIO_Init( DRV8301_EN_GATE_PORT , &GPIO_Initure );
	
		DRV8301_EN_GATE_LOW;
	
}


