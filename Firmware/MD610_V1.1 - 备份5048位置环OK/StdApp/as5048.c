#include "as5048.h"

#include "spi.h"



#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"


volatile as5048_t senser_as5048;

void AS5048_Init(void)
{	
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AS5048_CS, ENABLE );
	
    GPIO_InitStructure.GPIO_Pin =  AS5048_CS_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(AS5048_CS_PORT,&GPIO_InitStructure);

    AS5048_HIGH;

}

void AS5048_Selected(void)
{
		AS5048_CS_PORT->BSRRL = AS5048_CS_Pin;
}

void AS5048_Unselected(void)
{
		AS5048_CS_PORT->BSRRH = AS5048_CS_Pin;
}

int as5048_singelread_angle(void)
{
    // 写数据 读数据
//		int16_t receive_buffer;

    AS5048_LOW;
    senser_as5048.reg = (uint16_t)(SPI1_ReadWrite16Bit(AS5048_ANGLE)&0x3fff);
    AS5048_HIGH;

//    as5048.reg = (int16_t)(receive_buffer&0x3fff);
//		as5048.ef = (receive_buffer&0x4000)>>13;

    return 1;
}

