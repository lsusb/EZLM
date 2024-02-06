#include "hqfoc_app.h"
#include "main.h"
#include "user_interface.h"
#include "motor_control_protocol.h"
#include <string.h>
#include "control.h"
#include "filter.h"
#include "bsp_tle5012.h"

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
extern MCT_Handle_t* pMCT[NBR_OF_MOTORS];
extern PID_Handle_t *pPIDIq[NBR_OF_MOTORS];
extern PID_Handle_t *pPIDId[NBR_OF_MOTORS];
extern PID_Handle_t *pPIDSpeed[NBR_OF_MOTORS];

static int16_t tle5012_pos_read(tle5012_pack_p tle5012, int8_t dir);
static void Enc_dir_detection(mt_control_p mtc,tle5012_pack_p tle5012);
static void Auto_curr_dir_detection(mt_control_p mtc);
static uint8_t Auto_mag_alignment(mt_control_p mtc);
void cal_speed(mt_control_p mtc);


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
		
  tle5012_pos_read(&MotorControl.tle5012, MotorControl.parameter.enc_dir);

  if (MotorControl.info.motor_init == 1)
  {
    if (MotorControl.info.enc_dir_check == 0) // �������������������Ƕ����������Ƿ���ͬ������ͬ�Ļ����ı��������������
      Enc_dir_detection(&MotorControl,&MotorControl.tle5012);
    else if (MotorControl.info.mag_alignment == 0)
      Auto_mag_alignment(&MotorControl); // ��Ƕ��Զ�����
    else if (MotorControl.info.current_direction_check == 0) // ���������򣬷�ֹ������������
      Auto_curr_dir_detection(&MotorControl);
	
    MotorControl.foc.hElAngle = MotorControl.tle5012.ElAngle + MotorControl.parameter.pos_compensation;
  }

  PWMC_GetPhaseCurrents(pwmcHandle[M1], &MotorControl.foc.Iab);
	
  MotorControl.foc.Ialphabeta = MCM_Clarke(MotorControl.foc.Iab);

  MotorControl.foc.Iqd = MCM_Park(MotorControl.foc.Ialphabeta, MotorControl.foc.hElAngle);

  // ����������Vd��Vq//
  if (MotorControl.info.motor_init && MotorControl.info.current_direction_check)
  {
    cal_speed(&MotorControl);

    MotorControl.foc.Vqd.d = MotorControl.parameter.current_direction * PI_Controller(pPIDId[M1], (int32_t)(0) - MotorControl.foc.Iqd.d);

    MotorControl.foc.Vqd.q = MotorControl.parameter.current_direction * PI_Controller(pPIDIq[M1], (int32_t)(MotorControl.vel.output) - MotorControl.foc.Iqd.q);
  }

  MotorControl.foc.Vqd = Circle_Limitation(pCLM[M1], MotorControl.foc.Vqd);

  // hElAngle += SPD_GetInstElSpeedDpp(speedHandle) * REV_PARK_ANGLE_COMPENSATION_FACTOR;

  MotorControl.foc.Valphabeta = MCM_Rev_Park(MotorControl.foc.Vqd, MotorControl.foc.hElAngle);

  hCodeError = PWMC_SetPhaseVoltage(pwmcHandle[M1], MotorControl.foc.Valphabeta);
  
	Tle5012_DMA_read();

  return (hCodeError);
}


float get_Vbus_voltage(void)
{
	uint16_t bRetVal = (int32_t)VBS_GetAvBusVoltage_d(pMCT[M1]->pBusVoltageSensor);
	float bus_v = (bRetVal * pMCT[M1]->pBusVoltageSensor->ConversionFactor) / 65536.0f;
	return bus_v;
}


static int64_t overloop_proccess(int32_t single_pos, int32_t single_max)
{
  static int32_t single_pos_last = 0;
  static int32_t cnt = 0;

  if (single_pos_last - single_pos > single_max / 2)
    cnt++;
  if (single_pos_last - single_pos < -single_max / 2)
    cnt--;
  int64_t overloop_pos = single_pos + cnt * single_max;
  single_pos_last = single_pos;

  return overloop_pos;
}



static int16_t tle5012_pos_read(tle5012_pack_p tle5012, int8_t dir)
{

  uint16_t pos_data_temp = encoder_read();

  if (dir == -1)
    tle5012->posdata = -pos_data_temp + 65535;
  else
    tle5012->posdata = pos_data_temp;

  tle5012->overloop_pos = overloop_proccess(tle5012->posdata, 65536);

  tle5012->overloop_pos_filter = SlideAverageFilter(tle5012->overloop_pos);

  // as5311.pos_mm = as5311.overloop_pos / ENC_COUNT_PER_MM;

  // float pole_pair = 2 * POLE_PITCH;
  // float eangle = (as5311.pos_mm - floor(as5311.pos_mm / pole_pair) * pole_pair) / (pole_pair);
  // eangle = LIMIT(eangle,0,1);

  tle5012->ElAngle = (int16_t)((tle5012->overloop_pos * 14));

  return tle5012->ElAngle;
}

#define EDD_VD_VOLTAGE 3000                            // ����������̽�⣬Vd��ѹ
#define EDD_STABLE_TIME_MS 1000 * PWM_FREQUENCY / 1000 // ����������̽�⣬�ȴ�����Сʱ��
#define EDD_TEST_DISTANCE 65536 / 14

/// @brief ̽����������������Ƕ��������Ƿ�һ��
/// @param mtc
static void Enc_dir_detection(mt_control_p mtc,tle5012_pack_p tle5012)
{
  static uint8_t edd_stage = 0;
  static uint32_t time_cnt = 0;
  static int32_t pos_last = 0;

  if (edd_stage == 0)
  {
    mtc->foc.Vqd.d = EDD_VD_VOLTAGE;
    mtc->parameter.pos_compensation = -tle5012->ElAngle; // ����Ƕ�����Ϊ0  Eangle = mtc->parameter.pos_compensation + tle5012->ElAngle = 0

    if (++time_cnt > EDD_STABLE_TIME_MS)
    {
      time_cnt = 0;
      edd_stage = 1;
      pos_last = tle5012->overloop_pos;
    }
  }
  if (edd_stage == 1)
  {
    mtc->parameter.pos_compensation += 10;
    int32_t delta_pos = tle5012->overloop_pos - pos_last;
    if (myABS(delta_pos) > EDD_TEST_DISTANCE)
    {
      if (delta_pos < 0)
        mtc->parameter.enc_dir = -1; // �����������������Ƕ����������෴

      mtc->foc.Vqd.d = 0;
      mtc->info.enc_dir_check = 1; // ��ɱ�����������
      edd_stage = 0;
    }
  }
}

#define CDT_VD_VOLTAGE 20000                            // ���������⣬Vd��ѹ
#define CDT_STABLE_TIME_MS 50 * PWM_FREQUENCY / 1000 // ���������⣬�ȴ������ȶ�����Сʱ��

static void Auto_curr_dir_detection(mt_control_p mtc)
{
  static uint8_t cdt_stage = 0;
  static uint32_t time_cnt = 0;
  static qd_t Iqd_last = {0, 0};
  static int32_t pos_last = 0;

  if (cdt_stage == 0) // ��һ�׶Σ���¼VdΪ0ʱ��Idֵ
  {
    mtc->foc.Vqd.q = 0;
    if (++time_cnt > CDT_STABLE_TIME_MS)
    {
      time_cnt = 0;
      Iqd_last = mtc->foc.Iqd;
      cdt_stage = 1;
    }
  }
  else if (cdt_stage == 1) // �ڶ��׶Σ�����Vd�����Id�ı仯����Id-Id_last�ľ���ֵ������ֵʱ����ʼ���Id-Id_last�ķ����Ƿ�Ϊ��
  {
    mtc->foc.Vqd.q += 1;
    mtc->foc.Vqd.q = LIMIT(mtc->foc.Vqd.q, 0, CDT_VD_VOLTAGE);
    if (myABS(mtc->foc.Iqd.q - Iqd_last.q) > 800)
    {
      if (mtc->foc.Iqd.q - Iqd_last.q < 0)
        mtc->parameter.current_direction = -1;
      else
        mtc->parameter.current_direction = 1;

      mtc->foc.Vqd.q = 0;
      cdt_stage = 3;
    }
  }
  else if (cdt_stage == 3) // �����׶Σ�Vq=0���ȴ�������ȫֹͣ
  {
    mtc->foc.Vqd.q = 0;
    if (mtc->tle5012.overloop_pos - pos_last < 10)
    {
      if (++time_cnt > CDT_STABLE_TIME_MS)
      {
        time_cnt = 0;
        cdt_stage = 0;
        mtc->info.current_direction_check = 1;
      }
    }
    else
      time_cnt = 0;

    pos_last = mtc->tle5012.overloop_pos;
  }
}


#define KEEP_SPEED_ZERO_TIME_MS 1000 * PWM_FREQUENCY / 1000 // 0��Ƕȶ��룬���ӱ��־�ֹ����Сʱ��
#define ALIGNMENT_VD_VOLTAGE 3000                           // ��������е�Vd��ѹ
#define ALIGNMENT_MOVE_DISTANCE 2 * ENC_COUNT_PER_MM        // ����󣬲�����������ƶ�����

/// @brief ��Ƕ��Զ����룬���ڷ�ȫ�г̾���ֵ����������Ҫÿ���ϵ����ж��롣
/// @param pos ����λ�ã���������λ
/// @param fake_eangle
static uint8_t Auto_mag_alignment(mt_control_p mtc)
{
  static uint8_t alignment_stage = 0;
  static int32_t alignment_pos = 0, delta_pos = 0;
  static uint16_t stable_cnt = 0;

  if (alignment_stage == 0)
  {
    mtc->parameter.pos_compensation = -mtc->tle5012.ElAngle; // ����Ƕ�����Ϊ0  Eangle = mtc->parameter.pos_compensation + fake_eangle = 0

    mtc->foc.Vqd.d = ALIGNMENT_VD_VOLTAGE; // ����Vd��ѹΪALIGNMENT_VD_VOLTAGE

    delta_pos = mtc->tle5012.overloop_pos - alignment_pos;
    if (myABS(delta_pos) < 10) // �������λ�ñ仯С��10����������λ���ұ���KEEP_SPEED_ZERO_TIME_MSʱ�䣬˵�����Ӿ�ֹ����Ϊ�������
    {
      if (++stable_cnt > KEEP_SPEED_ZERO_TIME_MS)
      {
        alignment_stage = 1;
        mtc->foc.Vqd.d = 0;
        stable_cnt = 0;
      }
    }
    else
      stable_cnt = 0;
    alignment_pos = mtc->tle5012.overloop_pos; // ��¼����ʱ�Ķ���λ��
  }

  if (alignment_stage == 1)
  {
    mtc->foc.Vqd.q += 1;
    mtc->foc.Vqd.q = LIMIT(mtc->foc.Vqd.q, 0, ALIGNMENT_VD_VOLTAGE);
    delta_pos = mtc->tle5012.overloop_pos - alignment_pos;

    if (myABS(delta_pos) > ALIGNMENT_MOVE_DISTANCE) // �����������ƶ�ALIGNMENT_MOVE_DISTANCE����������λ
    {
      if (delta_pos < 0) // �������λ�ñ仯С��0��˵�����ӷ����˶�����pos_compensation�ƶ�180��Ƕȡ�
        mtc->parameter.pos_compensation += 32768;

      stable_cnt = 0;
      mtc->foc.Vqd.q = 0;
      alignment_stage = 0;
      mtc->info.mag_alignment = 1;
      return 1; // �������
    }
  }
  return 0;
}

void set_pid(void)
{
  PID_SetKP(pPIDIq[M1], 2000);
  PID_SetKI(pPIDIq[M1], 500);

  PID_SetKP(pPIDId[M1], 2000);
  PID_SetKI(pPIDId[M1], 500);

  PID_SetKP(pPIDSpeed[M1], 0);
  PID_SetKI(pPIDSpeed[M1], 0);
}

int16_t factor_yyy = 20;
void speed_loop_cal(mt_control_p mtc, float T)
{
  static int16_t cnt = 0;
  float result = 0;

  if (++cnt > 1)
  {
    MotorControl.vel.feedbk_vel = mtc->tle5012.speed_ori;
//    MotorControl.vel.set_vel = 0;
//    int32_t hError = MotorControl.vel.set_vel - MotorControl.vel.feedbk_vel;

//    MotorControl.vel.output = PI_Controller(pPIDSpeed[M1], ( int32_t )hError ) * factor_yyy;

    Speed_Control(&MotorControl.vel.set_vel, &MotorControl.vel.feedbk_vel, &result, T);

    MotorControl.vel.output = result;
    cnt = 0;
  }
}



void cal_speed(mt_control_p mtc)
{
  static int16_t cnt = 0;
  static int32_t pos_last = 0;
  static bool init = 0;

  if (init == 0)
  {
    pos_last = mtc->tle5012.overloop_pos_filter;
    init = 1;
    return;
  }

  if (++cnt > 1)
  {
    if (pos_last)
      mtc->tle5012.speed_ori = mtc->tle5012.overloop_pos_filter - pos_last;
    pos_last = mtc->tle5012.overloop_pos_filter;
    cnt = 0;

    speed_loop_cal(mtc, 0.001);
  }
}

