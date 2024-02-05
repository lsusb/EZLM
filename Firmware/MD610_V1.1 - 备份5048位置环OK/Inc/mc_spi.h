#ifndef __MC_SPI_H
#define __MC_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "board.h"

void SPI1_Init(void);

uint16_t SPI1_ReadWriteByte(uint16_t TxData);
void SPI1_SetSpeed(u8 SPI_BaudRatePrescaler);
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);
void DMA2_Stream2_IRQHandler(void);
void DMA2_Stream5_IRQHandler(void);

extern SPI_HandleTypeDef SPI1_Handler;  //SPI1¾ä±ú
extern DMA_HandleTypeDef SPI1RxDMA_Handler;
extern DMA_HandleTypeDef SPI1TxDMA_Handler;


#ifdef __cplusplus
}
#endif

#endif /* __MC_SPI_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
