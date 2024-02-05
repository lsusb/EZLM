/**
  ******************************************************************************
  * @file    stm32f4xx_mc_it.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Main Interrupt Service Routines.
  *          This file provides exceptions handler and peripherals interrupt
  *          service routine related to Motor Control for the STM32F4 Family.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  * @ingroup STM32F4xx_IRQ_Handlers
  */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "mc_config.h"
#include "mc_tasks.h"
#include "ui_task.h"
#include "parameters_conversion.h"
#include "motorcontrol.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"

/* USER CODE BEGIN Includes */
#include "schedule.h"

/* USER CODE END Includes */

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup STM32F4xx_IRQ_Handlers STM32F4xx IRQ Handlers
  * @{
  */

/* USER CODE BEGIN PRIVATE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SYSTICK_DIVIDER (SYS_TICK_FREQUENCY/1000)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* USER CODE END PRIVATE */

/* Public prototypes of IRQ handlers called from assembly code ---------------*/
void ADC_IRQHandler(void);
void TIMx_UP_M1_IRQHandler(void);
void TIMx_BRK_M1_IRQHandler(void);
void SPD_TIM_M1_IRQHandler(void);
void HardFault_Handler(void);
void SysTick_Handler(void);

/**
  * @brief  This function handles ADC1/ADC2 interrupt request.
  * @param  None
  * @retval None
  */
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */

  /* USER CODE END ADC_IRQn 0 */
  if(LL_ADC_IsActiveFlag_JEOS(ADC1))
  {
    // Clear Flags
    ADC1->SR &= ~(uint32_t)(LL_ADC_FLAG_JEOS | LL_ADC_FLAG_JSTRT);

    TSK_HighFrequencyTask();          /*GUI, this section is present only if DAC is disabled*/
  }
#ifdef ADC3
  else
  {
    // Clear Flags
    ADC3->SR &= ~(uint32_t)(LL_ADC_FLAG_JEOS | LL_ADC_FLAG_JSTRT);

    TSK_HighFrequencyTask();          /*GUI, this section is present only if DAC is disabled*/
  }
#endif
  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}

/**
  * @brief  This function handles first motor TIMx Update interrupt request.
  * @param  None
  * @retval None
  */
void TIMx_UP_M1_IRQHandler(void)
{
  /* USER CODE BEGIN TIMx_UP_M1_IRQn 0 */

  /* USER CODE END TIMx_UP_M1_IRQn 0 */

  LL_TIM_ClearFlag_UPDATE(PWM_Handle_M1.pParams_str->TIMx);
  ICS_TIMx_UP_IRQHandler(&PWM_Handle_M1);
  /* USER CODE BEGIN TIMx_UP_M1_IRQn 1 */

  /* USER CODE END TIMx_UP_M1_IRQn 1 */
}

/**
  * @brief  This function handles first motor BRK interrupt.
  * @param  None
  * @retval None
  */
void TIMx_BRK_M1_IRQHandler(void)
{
  /* USER CODE BEGIN TIMx_BRK_M1_IRQn 0 */

  /* USER CODE END TIMx_BRK_M1_IRQn 0 */
  if (LL_TIM_IsActiveFlag_BRK(PWM_Handle_M1.pParams_str->TIMx))
  {
    LL_TIM_ClearFlag_BRK(PWM_Handle_M1.pParams_str->TIMx);
    ICS_BRK_IRQHandler(&PWM_Handle_M1);
  }
  /* Systick is not executed due low priority so is necessary to call MC_Scheduler here.*/
  MC_Scheduler();

  /* USER CODE BEGIN TIMx_BRK_M1_IRQn 1 */

  /* USER CODE END TIMx_BRK_M1_IRQn 1 */
}

/**
  * @brief  This function handles TIMx global interrupt request for M1 Speed Sensor.
  * @param  None
  * @retval None
  */
void SPD_TIM_M1_IRQHandler(void)
{
  /* USER CODE BEGIN SPD_TIM_M1_IRQn 0 */

  /* USER CODE END SPD_TIM_M1_IRQn 0 */

 /* Encoder Timer UPDATE IT is dynamicaly enabled/disabled, checking enable state is required */
  if (LL_TIM_IsEnabledIT_UPDATE (ENCODER_M1.TIMx) && LL_TIM_IsActiveFlag_UPDATE (ENCODER_M1.TIMx))
  {
    LL_TIM_ClearFlag_UPDATE(ENCODER_M1.TIMx);
    ENC_IRQHandler(&ENCODER_M1);
    /* USER CODE BEGIN M1 ENCODER_Update */

    /* USER CODE END M1 ENCODER_Update   */
  }
  else
  {
  /* No other IT to manage for encoder config */
  }
  /* USER CODE BEGIN SPD_TIM_M1_IRQn 1 */

  /* USER CODE END SPD_TIM_M1_IRQn 1 */
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
 /* USER CODE BEGIN HardFault_IRQn 0 */

 /* USER CODE END HardFault_IRQn 0 */
  TSK_HardwareFaultTask();

  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
 /* USER CODE BEGIN HardFault_IRQn 1 */

 /* USER CODE END HardFault_IRQn 1 */

}

void SysTick_Handler(void)
{

#ifdef MC_HAL_IS_USED
static uint8_t SystickDividerCounter = SYSTICK_DIVIDER;
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  if (SystickDividerCounter == SYSTICK_DIVIDER)
  {
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
    SystickDividerCounter = 0;
  }
  SystickDividerCounter ++;
#endif /* MC_HAL_IS_USED */

  /* USER CODE BEGIN SysTick_IRQn 1 */
  HQ_SYSTICK_IRQHandler();
  /* USER CODE END SysTick_IRQn 1 */
    MC_RunMotorControlTasks();

  /* USER CODE BEGIN SysTick_IRQn 2 */
  /* USER CODE END SysTick_IRQn 2 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */
/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
