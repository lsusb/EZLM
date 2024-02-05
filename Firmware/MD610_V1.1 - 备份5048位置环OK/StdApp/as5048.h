#ifndef _AS5048_H_
#define _AS5048_H_
#include "std_config.h"




#define  RCC_AS5048_CS           RCC_AHB1Periph_GPIOA
#define  AS5048_CS_PORT          GPIOA
#define  AS5048_CS_Pin     			 GPIO_Pin_4

#define  AS5048_LOW       	 			AS5048_CS_PORT->BSRRH = AS5048_CS_Pin
#define  AS5048_HIGH      				AS5048_CS_PORT->BSRRL = AS5048_CS_Pin
#define  AS5048_TOGGLE    				AS5048_CS_PORT->ODR ^= AS5048_CS_Pin

#define AS5048_ANGLE					0xFFFF

typedef struct _as5048
{
    uint16_t       reg;
		uint16_t			 reg_cal;
		long long			 pos;
		long 					 cnt;
		int16_t       	 ElectricAngle;
		
    u8     ef;
    long pos_sum;
		float angle;
    float       speed;
    float       speed_ef;
    long int    speed_jscop;
} as5048_t;


void AS5048_Init(void);
int as5048_singelread_angle(void);
void AS5048_Selected(void);
void AS5048_Unselected(void);

extern volatile as5048_t senser_as5048;

#endif
