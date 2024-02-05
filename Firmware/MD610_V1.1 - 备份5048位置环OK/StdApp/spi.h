#ifndef __SPI_H
#define __SPI_H

#include "std_config.h"
//#include "std_stm32f4xx.h"
//#include "stm32f4xx_conf.h"

#define  RCC_SPI3_CS           RCC_AHB1Periph_GPIOC
#define  SPI3_CS_PORT          GPIOC
#define  SPI3_CS_Pin     			 GPIO_Pin_9

#define  SPI3_CS_LOW       	 			SPI3_CS_PORT->BSRRH = SPI3_CS_Pin
#define  SPI3_CS_HIGH      				SPI3_CS_PORT->BSRRL = SPI3_CS_Pin
#define  SPI3_CS_TOGGLE    				SPI3_CS_PORT->ODR ^= SPI3_CS_Pin

void SPI1_Init(void);			 //初始化SPI1口
void SPI1_SetSpeed(u8 SpeedSet); //设置SPI1速度
uint16_t SPI1_ReadWrite16Bit(uint16_t Txdata);
void Spi1_Function(int OutputBuff[],int16_t InputBuff[], unsigned int NoOfBytes);
void SPI1_TX_DMA_Config(void);
void SPI1_RX_DMA_Config(void);
void SPI_DMA_WRITE_READ_BUF(void);

void SPI3_Init(void);

uint16_t SPI3_ReadWrite16Bit(uint16_t Txdata);


#endif
