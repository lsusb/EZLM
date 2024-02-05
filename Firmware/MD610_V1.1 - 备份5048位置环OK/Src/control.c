#include "control.h"
#include "spi.h"

#include "mc_svpwm.h"
#include "my_math.h"
#include "parameter.h"

#include "pwm_curr_fdbk.h"
#include "mc_tasks.h"
#include "mc_math.h"
#include "as5048.h"

#include "mc_type.h"


_mt_control MotorControl;

void Foc_Control(void)
{
		
	//		mc_svpm.Park.Uq = 8.0f;
		ab_t Iab;
		alphabeta_t Ialphabeta, Valphabeta;
		int16_t hElAngle;
		qd_t Iqd, Vqd;
		
		float traget_id = 0;
		float feedbk_id = 0;
		float result_id = 0;

		float feedbk_iq = 0;
		float result_iq = 0;
		
		SPI_DMA_WRITE_READ_BUF();

		GetPhaseCurrents();
		
		Iab.a =  mc_svpm.Clack.Ia;
		Iab.b =  mc_svpm.Clack.Ib;
		
		Ialphabeta = MCM_Clarke(Iab);
		
		hElAngle = senser_as5048.ElectricAngle;

		Iqd = MCM_Park(Ialphabeta, hElAngle);

		feedbk_id = LPF2pApply(0,Iqd.d); 
		feedbk_iq = LPF2pApply(1,Iqd.q);
		
//		Current_IdControl(&traget_id,&feedbk_id,&mc_svpm.Park.Ud,0.0625f);
//		Current_IqControl(&MotorControl.vel.output,&feedbk_iq,&mc_svpm.Park.Uq,0.0625f);

		Vqd.q = mc_svpm.Park.Uq;
		Vqd.d = mc_svpm.Park.Ud;
		
		Vqd = Circle_Limitation(pCLM[M1], Vqd);
		
		Valphabeta = MCM_Rev_Park(Vqd, hElAngle);
		
		PWMC_SetPhaseVoltage(pwmcHandle[M1], Valphabeta);

//		GetPhaseCurrents();
//		
//		Clarke_Trans();
//	
//		Park_Trans();
//	
//		CurrentControl();
//	
//		Anti_Park_Calc();
//	
//		Svpwm_Module();
	
	
	
	
}




void CurrentControl(void)
{
		float traget_id = 0;
		float feedbk_id = mc_svpm.Park.Id;
		float result_id = 0;

		float feedbk_iq = mc_svpm.Park.Iq;
		float result_iq = 0;
	
		Current_IdControl(&traget_id,&feedbk_id,&result_id,0.0625f);
		Current_IqControl(&MotorControl.vel.output,&feedbk_iq,&result_iq,0.0625f);
	
		mc_svpm.Park.Ud = result_id;
		mc_svpm.Park.Uq = result_iq;
}


void Current_IdControl(float* setValue, float* fbValue, int16_t* result, float T)
{
    static pid currentId;

    static u8 paraLoadFlag = 0;

    /* �������û�м��أ����ز���*/
    if(!paraLoadFlag)
    {
        Get_PIDparameters(&currentId, pidCurrent_Id);
//        paraLoadFlag = 1;
    }

    /* �趨ֵ */
    currentId.setValue = *setValue;
    /* ����ֵ */
    currentId.feedbackValue = *fbValue;
    /* ƫ�� = �趨ֵ - ����ֵ */
    currentId.error = currentId.setValue - currentId.feedbackValue;
    /* ���������� */
    currentId.pOut = currentId.kp * currentId.error;
    /* ƫ����л��� */
    currentId.integralError += currentId.error * T;
    /* ƫ��Ļ��ֽ������� */
    currentId.integralError = LIMIT(currentId.integralError, -currentId.integralErrorMax, currentId.integralErrorMax);
    /* ���������� */
    currentId.iOut = currentId.ki * currentId.integralError;
    /* �ܵ���� = ���������� */
    currentId.out = currentId.pOut + currentId.iOut;
    /* �ܵ�������ܳ����������ֵ�ķ�Χ */
    *result = LIMIT(currentId.out, -32767, 32767);

}

 

void Current_IqControl(float* setValue, float* fbValue, int16_t* result, float T)
{
    static pid currentIq;

    static u8 paraLoadFlag = 0;

    /* �������û�м��أ����ز���*/
    if(!paraLoadFlag)
    {
        Get_PIDparameters(&currentIq, pidCurrent_Iq);
//        paraLoadFlag = 1;
    }

    /* �趨ֵ */
    currentIq.setValue = *setValue;
    /* ����ֵ */
    currentIq.feedbackValue = *fbValue;
    /* ƫ�� = �趨ֵ - ����ֵ */
    currentIq.error = currentIq.setValue - currentIq.feedbackValue;
    /* ���������� */
    currentIq.pOut = currentIq.kp * currentIq.error;
    /* ƫ����л��� */
    currentIq.integralError += currentIq.error * T;
    /* ƫ��Ļ��ֽ������� */
    currentIq.integralError = LIMIT(currentIq.integralError, -currentIq.integralErrorMax, currentIq.integralErrorMax);
    /* ���������� */
    currentIq.iOut = currentIq.ki * currentIq.integralError;
    /* �ܵ���� = ���������� */
    currentIq.out = currentIq.pOut + currentIq.iOut;
    /* �ܵ�������ܳ����������ֵ�ķ�Χ */
    *result = LIMIT(currentIq.out, -32767, 32767);

}



void MotorControlApp(float T)
{
		static u8 cal_pos = 0;
	
		static int pos_last,deta_pos;
	
		deta_pos = senser_as5048.reg - pos_last;
		if(deta_pos < -16384/2)
			MotorControl.vel.reg_speed = deta_pos + 16384;
		else if(deta_pos > 16384/2)
			MotorControl.vel.reg_speed = deta_pos - 16384;
		else
			MotorControl.vel.reg_speed = deta_pos;
		
		pos_last = senser_as5048.reg;
		
		MotorControl.pos.feedbk_pos = senser_as5048.pos;
		
		MotorControl.vel.feedbk_vel = LPF2pApply(2,MotorControl.vel.reg_speed);
		

		Speed_Control(&MotorControl.vel.set_vel,&MotorControl.vel.feedbk_vel,&MotorControl.vel.output,T);
		
		if(++cal_pos > 2)
		{
				Position_Control(&MotorControl.pos.set_pos,&MotorControl.pos.feedbk_pos,&MotorControl.vel.set_vel,T);
				cal_pos = 0;
		}
		
//		MotorControl.vel.output = 200;
}

void Speed_Control(float* setValue, float* fbValue, float* result, float T)
{
    static pid mc_speed;

    static u8 paraLoadFlag = 0;

    /* �������û�м��أ����ز���*/
    if(!paraLoadFlag)
    {
        Get_PIDparameters(&mc_speed, pidSpeed);
//        paraLoadFlag = 1;
    }

    /* �趨ֵ */
    mc_speed.setValue = *setValue;
    /* ����ֵ */
    mc_speed.feedbackValue = *fbValue;
    /* ƫ�� = �趨ֵ - ����ֵ */
    mc_speed.error = mc_speed.setValue - mc_speed.feedbackValue;
    /* ���������� */
    mc_speed.pOut = mc_speed.kp * mc_speed.error;
    /* ƫ����л��� */
    mc_speed.integralError += mc_speed.error * T;
    /* ƫ��Ļ��ֽ������� */
    mc_speed.integralError = LIMIT(mc_speed.integralError, -mc_speed.integralErrorMax, mc_speed.integralErrorMax);
    /* ���������� */
    mc_speed.iOut = mc_speed.ki * mc_speed.integralError;
    /* �ܵ���� = ���������� */
    mc_speed.out = mc_speed.pOut + mc_speed.iOut;
    /* �ܵ�������ܳ����������ֵ�ķ�Χ */
    *result = LIMIT(mc_speed.out, -2047, 2047);
}



void Position_Control(float* setValue, float* fbValue, float* result, float T)
{
    static pid mc_position;

    static u8 paraLoadFlag = 0;

    /* �������û�м��أ����ز���*/
    if(!paraLoadFlag)
    {
        Get_PIDparameters(&mc_position, pidPos);
//        paraLoadFlag = 1;
    }

    /* �趨ֵ */
    mc_position.setValue = *setValue;
    /* ����ֵ */
    mc_position.feedbackValue = *fbValue;
    /* ƫ�� = �趨ֵ - ����ֵ */
    mc_position.error = mc_position.setValue - mc_position.feedbackValue;
    /* ���������� */
    mc_position.pOut = mc_position.kp * mc_position.error;
    /* ƫ����л��� */
    mc_position.integralError += mc_position.error * T;
    /* ƫ��Ļ��ֽ������� */
    mc_position.integralError = LIMIT(mc_position.integralError, -mc_position.integralErrorMax, mc_position.integralErrorMax);
    /* ���������� */
    mc_position.iOut = mc_position.ki * mc_position.integralError;
    /* �ܵ���� = ���������� */
    mc_position.out = mc_position.pOut + mc_position.iOut;
    /* �ܵ�������ܳ����������ֵ�ķ�Χ */
    *result = LIMIT(mc_position.out, -450, 450);
}



