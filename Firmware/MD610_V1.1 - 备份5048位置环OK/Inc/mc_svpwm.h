#ifndef __MC_SVPWM_H
#define __MC_SVPWM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"


#define PhaseA 0
#define PhaseB 1
#define PhaseC 2
typedef struct 
{
    long Ia;     //���룬A�ඨ�ӵ���
    long Ib;     //���룬B�ඨ�ӵ���
    long Ic;     //���룬C�ඨ�ӵ���
    float IAlpha;  //�������ֹ����ϵAlpha�ᶨ�ӵ���
    float IBeta;   //�������ֹ����ϵBeta�ᶨ�ӵ���
    void (*calcClark)();    
    void (*calcAntiClark)();
}_CLARK;

typedef struct 
{
    float Id;     //�������ת����ϵ�µ�D����ֵ����
    float Iq;         //�������ת����ϵ�µ�Q����ֵ����
    float IAlpha;  //���룬��ֹ����ϵAlpha�ᶨ�ӵ���
    float IBeta;   //���룬��ֹ����ϵBeta�ᶨ�ӵ���
    int16_t Ud;     //�������ת����ϵ�µ�D����ֵ��ѹ
    int16_t Uq;          //�������ת����ϵ�µ�Q����ֵ��ѹ
    float UAlpha;  //���룬��ֹ����ϵAlpha�ᶨ�ӵ�ѹ
    float UBeta;   //���룬��ֹ����ϵBeta�ᶨ�ӵ�ѹ
    long Theta;    //��ת����Ƕ�
    float ActId;  //ʵ��D�����
    float ActIq;  //ʵ��Q�����
    void (*calcPark)();      
    void (*calcAntiPark)();  
}_PARK;

typedef struct 
{
    float UAlpha; //���룬��ֹ����ϵAlpha�ᶨ�ӵ�ѹ
    float UBeta;  //���룬��ֹ����ϵBeta�ᶨ�ӵ�ѹ
    float Ua;      //
    float Ub;      //
    float Uc;      //
    float Tx;        //
    float Ty;        //
    float Tz;        //
    float taOn;    //A��ʱ��
    float tbOn;      //B��ʱ��
    float tcOn;      //C��ʱ��
} _SVPWM;

typedef struct  _phase{
                   int   H_duty;
                   int   L_duty;   
}phase;

typedef struct {
                 int MOTA;
                 int MOTB;
                 int MOTC;  
}_duty;


typedef struct {
                 uint16_t reg[3];
                 uint32_t offeset[3];
								 volatile uint16_t	sample_cnt;
								 volatile _Bool    Calibration;
}_dac;

typedef struct{
               _duty  Duty;
               _SVPWM Svpwm;
               _CLARK Clack;
               _PARK  Park;
							 _dac		AdcData;
               
}_MCPWM_FOC;

#define MOTOR_POWER   24 
#define Ts            4000
#define Time1_Period  4000
#define MAX_Duty      4000


//void mcpwm_foc_init(void);
void Svpwm_Module(void);
void Foc_Control(void);
void SwitchOnPWM(void);
void SwitchOffPWM(void);
void GetPhaseCurrents(void);


extern _MCPWM_FOC mc_svpm;





#ifdef __cplusplus
}
#endif

#endif /* __MC_SVPWM_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
