#include "bsp_tle5012.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;

#define TLE_SPI 		hspi1
#define TLE_NSS_HEIH	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET)
#define TLE_NSS_LOW		HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET)


#define SEN_CNT 2           // connection: MCU_MOSI -- resistor -- MCU_MISO -- TLE5012B_DATA


volatile uint16_t sen_rx_val[2] = {0};
static uint16_t sen_tx_val[2] = {0x8021, 0};
uint16_t ang_reg_v = 0x8021, data_v;


uint16_t encoder_read(void)
{
//    return 0xffff - (sen_rx_val[1] << 1);
	
    return 0xffff - (data_v << 1);
}

void encoder_isr_prepare(SPI_HandleTypeDef* spi)
{
    __HAL_DMA_DISABLE(spi->hdmarx);
    spi->hdmarx->Instance->NDTR = SEN_CNT;
    spi->hdmarx->Instance->PAR = (uint32_t)&spi->Instance->DR;
    spi->hdmarx->Instance->M0AR = (uint32_t)sen_rx_val;
    __HAL_DMA_ENABLE(spi->hdmarx);

    __HAL_DMA_DISABLE(spi->hdmatx);
    spi->hdmatx->Instance->NDTR = SEN_CNT;
    spi->hdmatx->Instance->PAR = (uint32_t)&spi->Instance->DR;
    spi->hdmatx->Instance->M0AR = (uint32_t)sen_tx_val;
}

void Init_tle5012(void)
{
//    SET_BIT(TLE_SPI.Instance->CR2, SPI_CR2_RXDMAEN);
//    SET_BIT(TLE_SPI.Instance->CR2, SPI_CR2_TXDMAEN);
//    encoder_isr_prepare(&TLE_SPI);
//    __HAL_SPI_ENABLE(&TLE_SPI);	
}


void Tle5012_DMA_read(void)
{
//	encoder_isr_prepare(&TLE_SPI);
//	TLE_NSS_LOW;
//	__HAL_DMA_ENABLE(TLE_SPI.hdmatx);
//	
//	TLE_NSS_LOW;
//	HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)&sen_tx_val, (uint8_t *)&sen_rx_val,2);
	
	TLE_NSS_LOW;
	HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)(&ang_reg_v), 1);
	HAL_SPI_Receive_DMA(&hspi1, (uint8_t *)(&data_v), 1);
}





void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if(hspi->Instance == TLE_SPI.Instance)
	  TLE_NSS_HEIH;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if(hspi->Instance == TLE_SPI.Instance)
	  TLE_NSS_HEIH;	
}

