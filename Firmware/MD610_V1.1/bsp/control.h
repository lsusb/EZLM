#ifndef _CONTROL_H
#define _CONTROL_H

#include "std_config.h"
#include "mc_type.h"

typedef enum
{
	eeprom_err = 1

} my_State_t;

typedef struct
{
	float set_vel;
	float reg_speed;
	float feedbk_vel;
	float output;

	float speed_rpm;
} _mt_velcontrol;

typedef struct
{
	float set_pos;
	float feedbk_pos;
	float output;
} mt_poscontrol_t;

typedef struct
{
	float VBus;
	u8 motor_init;
	bool enc_dir_check; // 编码器方向检查，防止编码器方向错误
	u8 mag_alignment;	//电角度是否对齐
	bool current_direction_check; // 电机电流环方向检查，防止电流环正反馈，正确的方向定义：Vq增大->Iq增大
	bool eeprom_s;

	u8 eeprom_write;
	u8 eeprom_read;

	u8 can_id;

	u8 motor_driver_name; // 电调类型

	uint16_t version_number_eeprom;
	uint16_t version_number;

} _mt_info;

typedef struct
{
	int16_t kp;
	int16_t ki;
	int16_t li;
	int16_t kd;
} _pid_parameter;

typedef struct
{
	int8_t enc_dir;			 // 编码器方向
	int8_t current_direction; // 电流方向
	int16_t pos_compensation; // 电角度对齐补偿值

	u8 control_mode; // 0:力矩模式        1：速度模式          2：位置模式

	int16_t feed_back_position_zeropoint; // 机械零位偏置

	uint16_t max_angle;		 // 最大转角
	uint16_t max_speed;		 // 最大转速
	uint16_t feed_back_freq; // 反馈频率

	_pid_parameter speed_parameter;
	_pid_parameter pos_parameter;

} mt_parameter_t;

typedef struct
{
	long long pos;
	float pos_filter;
	float speed;
} _mt_magnetic_sensor;

typedef struct
{
	float v_error;
	float p_error;
} _mt_controller;

typedef struct
{

	qd_t Iqd;
	qd_t Iqd_f;

	qd_t Vqd;

	ab_t Iab;
	ab_t Iab_f;

	alphabeta_t Ialphabeta;
	alphabeta_t Valphabeta;

	int16_t hElAngle;
	int16_t hMecAngle;

} _mt_foc;

typedef struct as5311_pack
{
	_Bool even_par;
	_Bool mag_dec;
	_Bool mag_inc;
	_Bool lin;
	_Bool cof;
	_Bool ocf;

	int16_t posdata;
	int16_t magdata;
	int16_t ElAngle;

	int32_t overloop_pos;
	int32_t overloop_pos_filter;

	float pos_mm;

	int16_t speed_ori;

} as5311_pack_t, *as5311_pack_p;

typedef struct
{
	mt_poscontrol_t pos;
	_mt_velcontrol vel;
	_mt_info info;
	_mt_foc foc;
	_mt_magnetic_sensor tle5012;
	_mt_controller controller;
	mt_parameter_t parameter;
	as5311_pack_t as5311;

} mt_control_t, *mt_control_p;

typedef struct
{
	int32_t vel_start[7];
	int32_t vel_end[7];

	int32_t acc_start[7];
	int32_t acc_end[7];

	int32_t pos_deta[7];

	int16_t Max_Jerk;
	int32_t Max_Acc;
	int32_t Max_Vel;

} _mt_setValue_planer;

void Foc_Control(void);

void Current_IdControl(int16_t *setValue, int16_t *fbValue, int16_t *result, float T);
void Current_IqControl(float *setValue, int16_t *fbValue, int16_t *result, float T);
void MotorControlApp(float T);
void Speed_Control(float *setValue, float *fbValue, float *result, float T);
void Position_Control(float *setValue, float *fbValue, float *result, float T);

void GM3510_Speed_Control(float *setValue, float *fbValue, float *result, float T);
void GM3510_Pos_Control(float *setValue, float *fbValue, float *result, float T);

void GM3510_Control(float T);
float HardLIMIT(float rawdata, float min, float max);

void UsartSendFeed_back(void);

void set_value_planer(void);

int32_t myABS(int32_t value);

extern mt_control_t MotorControl;

#endif
