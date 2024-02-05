#include "parameter.h"


float kp_id = 100,ki_id = 50,kd_id,li_id = 600;
float kp_iq = 100,ki_iq = 50,kd_iq,li_iq = 600;
float kp_v = 3,ki_v = 0,kd_v,li_v = 0;
float kp_pos = 0.1,ki_pos = 0,kd_pos,li_pos = 0;

void Get_PIDparameters(pid* pidStruct, u8 pidIndex)
{
    switch(pidIndex)
    {

				case pidCurrent_Id:
				{
						pidStruct->kp = kp_id;//20
						pidStruct->ki = ki_id;//4;
						pidStruct->kd = kd_id;
						pidStruct->integralErrorMax = li_id;
						break;
				}

				case pidCurrent_Iq:
				{
						pidStruct->kp = kp_iq;//20
						pidStruct->ki = ki_iq;//4;
						pidStruct->kd = kd_iq;
						pidStruct->integralErrorMax = li_iq;
						break;
				}			

				case pidSpeed:
				{
						pidStruct->kp = kp_v;//20
						pidStruct->ki = ki_v;//4;
						pidStruct->kd = kd_v;
						pidStruct->integralErrorMax = li_v;
						break;
				}			

				case pidPos:
				{
						pidStruct->kp = kp_pos;//20
						pidStruct->ki = ki_pos;//4;
						pidStruct->kd = kd_pos;
						pidStruct->integralErrorMax = li_pos;
						break;
				}				
				
    default:
        break;
    }
}



