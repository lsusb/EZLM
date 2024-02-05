#include "board.h"

#include "spi.h"
#include "as5048.h"
#include "led.h"
#include "drv8301.h"
#include "mc_svpwm.h"
#include "my_math.h"
#include "control.h"

/***********************HAL库****************************/
#include "stm32f4xx_hal.h"
#include "parameters_conversion.h"
#include "stm32f4xx.h"


u8 Init_OK;


#define TICK_PER_SECOND 2000
#define TICK_US	(1000000/TICK_PER_SECOND)
volatile uint32_t sysTickUptime=0;

uint32_t GetSysTime_us(void)
{
    register uint32_t ms;
    u32 value;
    ms = sysTickUptime;
    value = ms * TICK_US + (SysTick->LOAD - SysTick->VAL) * TICK_US / SysTick->LOAD;
    return value;
}

void DelayUs(uint32_t us)
{
    uint32_t now = GetSysTime_us();
    while (GetSysTime_us() - now < us);
}

void DelayMs(uint32_t ms)
{
    while (ms--)
        DelayUs(1000);
}



void TDT_Board_ALL_Init(void)
{
		LED_Init();
		
		/*SPI1初始化*/
		SPI1_Init();
		
		/*AS5048初始化*/
		AS5048_Init();
	
		DRV8301_Init();
	
		SwitchOnPWM();
	
		CurrentsCalibration();
	
		lpf_k_init();
	
		senser_as5048.cnt = 0;
		senser_as5048.pos = 0;
		MotorControl.vel.set_vel = 0;
		MotorControl.pos.set_pos = 0;
		Init_OK = 1;
}
