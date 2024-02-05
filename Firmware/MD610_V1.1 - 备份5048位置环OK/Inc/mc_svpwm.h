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
    long Ia;     //输入，A相定子电流
    long Ib;     //输入，B相定子电流
    long Ic;     //输入，C相定子电流
    float IAlpha;  //输出，静止坐标系Alpha轴定子电流
    float IBeta;   //输出，静止坐标系Beta轴定子电流
    void (*calcClark)();    
    void (*calcAntiClark)();
}_CLARK;

typedef struct 
{
    float Id;     //输出，旋转坐标系下的D坐标值电流
    float Iq;         //输出，旋转坐标系下的Q坐标值电流
    float IAlpha;  //输入，静止坐标系Alpha轴定子电流
    float IBeta;   //输入，静止坐标系Beta轴定子电流
    int16_t Ud;     //输出，旋转坐标系下的D坐标值电压
    int16_t Uq;          //输出，旋转坐标系下的Q坐标值电压
    float UAlpha;  //输入，静止坐标系Alpha轴定子电压
    float UBeta;   //输入，静止坐标系Beta轴定子电压
    long Theta;    //旋转坐标角度
    float ActId;  //实际D轴电流
    float ActIq;  //实际Q轴电流
    void (*calcPark)();      
    void (*calcAntiPark)();  
}_PARK;

typedef struct 
{
    float UAlpha; //输入，静止坐标系Alpha轴定子电压
    float UBeta;  //输入，静止坐标系Beta轴定子电压
    float Ua;      //
    float Ub;      //
    float Uc;      //
    float Tx;        //
    float Ty;        //
    float Tz;        //
    float taOn;    //A相时间
    float tbOn;      //B相时间
    float tcOn;      //C相时间
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
