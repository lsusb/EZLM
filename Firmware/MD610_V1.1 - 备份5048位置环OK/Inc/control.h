#ifndef _CONTROL_H
#define	_CONTROL_H

#include "std_config.h"     


typedef struct{
		float set_vel;
		float reg_speed;
		float feedbk_vel;
		float output;
}_mt_velcontrol;

typedef struct{
		float set_pos;
		float feedbk_pos;
		float output;
}_mt_poscontrol;

typedef struct{
		float VBus;
}_mt_info;


typedef struct{
		_mt_poscontrol pos;
		_mt_velcontrol vel;
		_mt_info       info;
}_mt_control;


void CurrentControl(void);


void Current_IdControl(float* setValue, float* fbValue, int16_t* result, float T);
void Current_IqControl(float* setValue, float* fbValue, int16_t* result, float T);
void MotorControlApp(float T);
void Speed_Control(float* setValue, float* fbValue, float* result, float T);
void Position_Control(float* setValue, float* fbValue, float* result, float T);


extern _mt_control MotorControl;


#endif


