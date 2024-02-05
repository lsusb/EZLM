#include "hqfoc_app.h"
#include "main.h"
#include "user_interface.h"
#include "motor_control_protocol.h"
#include <string.h>
#include "control.h"
#include "filter.h"

#include "main.h"
#include "mc_type.h"
#include "mc_math.h"
#include "motorcontrol.h"
#include "regular_conversion_manager.h"
#include "mc_interface.h"
#include "mc_tuning.h"
#include "digital_output.h"
#include "state_machine.h"
#include "pwm_common.h"

#include "mc_tasks.h"
#include "parameters_conversion.h"

extern PWMC_Handle_t *pwmcHandle[NBR_OF_MOTORS];
extern CircleLimitation_Handle_t *pCLM[NBR_OF_MOTORS];
extern FOCVars_t FOCVars[NBR_OF_MOTORS];
extern SpeednTorqCtrl_Handle_t *pSTC[NBR_OF_MOTORS];


void FCP_SetClient(FCP_Handle_t *pHandle,
                   struct MCP_Handle_s *pClient,
                   FCP_SentFrameCallback_t pSentFrameCb,
                   FCP_ReceivedFrameCallback_t pReceviedFrameCb,
                   FCP_RxTimeoutCallback_t pRxTimeoutCb)
{
}


void drv_init(void)
{
	HAL_GPIO_WritePin(DRV8301_EN_GATE_GPIO_Port,DRV8301_EN_GATE_Pin,GPIO_PIN_SET);
}

uint8_t drv_fault_check(void)
{
	uint8_t fault = HAL_GPIO_ReadPin(DRV8301_FAULT_GPIO_Port,DRV8301_FAULT_Pin);
	return fault;
}

uint16_t HQ_FOC_CurrControllerM1(void)
{
  uint16_t hCodeError;

//  as5311_pos_read(&MotorControl.as5311, MotorControl.parameter.enc_dir);

//  if (MotorControl.info.motor_init == 1)
//  {
//    if (MotorControl.info.enc_dir_check == 0) // 检查编码器增量方向与电角度增量方向是否相同，不相同的话，改变编码器增量方向
//      Enc_dir_detection(&MotorControl);
//    else if (MotorControl.info.current_direction_check == 0) // 检查电流方向，防止电流环正反馈
//      Auto_curr_dir_detection(&MotorControl);
//    else if (MotorControl.info.mag_alignment == 0)
//      Auto_mag_alignment(&MotorControl); // 电角度自动对齐

//    MotorControl.foc.hElAngle = MotorControl.as5311.ElAngle + MotorControl.parameter.pos_compensation;
//  }

  PWMC_GetPhaseCurrents(pwmcHandle[M1], &MotorControl.foc.Iab);
	
  MotorControl.foc.Ialphabeta = MCM_Clarke(MotorControl.foc.Iab);

  MotorControl.foc.Iqd = MCM_Park(MotorControl.foc.Ialphabeta, MotorControl.foc.hElAngle);

  // 电流环计算Vd、Vq//
//  if (MotorControl.info.motor_init && MotorControl.info.mag_alignment)
//  {
////    cal_speed(&MotorControl);

//    MotorControl.foc.Vqd.d = MotorControl.parameter.current_direction * PI_Controller(pPIDId[M1], (int32_t)(0) - MotorControl.foc.Iqd.d);

//    MotorControl.foc.Vqd.q = MotorControl.parameter.current_direction * PI_Controller(pPIDIq[M1], (int32_t)(MotorControl.vel.output) - MotorControl.foc.Iqd.q);
//  }

  MotorControl.foc.Vqd = Circle_Limitation(pCLM[M1], MotorControl.foc.Vqd);

  // hElAngle += SPD_GetInstElSpeedDpp(speedHandle) * REV_PARK_ANGLE_COMPENSATION_FACTOR;

  MotorControl.foc.Valphabeta = MCM_Rev_Park(MotorControl.foc.Vqd, MotorControl.foc.hElAngle);

  hCodeError = PWMC_SetPhaseVoltage(pwmcHandle[M1], MotorControl.foc.Valphabeta);

  return (hCodeError);
}
