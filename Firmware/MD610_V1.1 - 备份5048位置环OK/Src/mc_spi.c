#include "mc_spi.h"


SPI_HandleTypeDef SPI1_Handler;  //SPI1���
DMA_HandleTypeDef SPI1RxDMA_Handler;
DMA_HandleTypeDef SPI1TxDMA_Handler;

//����DMAΪ����ģʽ
void SPI1_DMA_config(void)
{
		__HAL_RCC_DMA2_CLK_ENABLE();//DMA2ʱ��ʹ�� 
    __HAL_LINKDMA(&SPI1_Handler,hdmarx,SPI1RxDMA_Handler);              //��DMA��SPI2��ϵ����(����DMA)
	
    //Rx DMA����
    SPI1RxDMA_Handler.Instance       = DMA2_Stream2;                   		//������ѡ��
    SPI1RxDMA_Handler.Init.Channel   = DMA_CHANNEL_3;                  		//ͨ��ѡ��
    SPI1RxDMA_Handler.Init.Direction = DMA_PERIPH_TO_MEMORY;           		//���赽�洢��
    SPI1RxDMA_Handler.Init.PeriphInc = DMA_PINC_DISABLE;               		//���������ģʽ
    SPI1RxDMA_Handler.Init.MemInc    = DMA_MINC_ENABLE;                		//�洢������ģʽ
    SPI1RxDMA_Handler.Init.PeriphDataAlignment=DMA_MDATAALIGN_HALFWORD;	  //�������ݳ���:16λ
    SPI1RxDMA_Handler.Init.MemDataAlignment=DMA_MDATAALIGN_HALFWORD;   		//�洢�����ݳ���:16λ
    SPI1RxDMA_Handler.Init.Mode=DMA_NORMAL;                            		//��������ģʽ
    SPI1RxDMA_Handler.Init.Priority=DMA_PRIORITY_MEDIUM;               		//�е����ȼ�
    SPI1RxDMA_Handler.Init.FIFOMode=DMA_FIFOMODE_DISABLE;              
	
    HAL_DMA_DeInit(&SPI1RxDMA_Handler);   
    HAL_DMA_Init(&SPI1RxDMA_Handler);

		__HAL_LINKDMA(&SPI1_Handler,hdmatx,SPI1TxDMA_Handler);              //��DMA��SPI2��ϵ����(����DMA)
	
    //Tx DMA����
    SPI1TxDMA_Handler.Instance       = DMA2_Stream5;                   //������ѡ��
    SPI1TxDMA_Handler.Init.Channel   = DMA_CHANNEL_3;                  //ͨ��ѡ��
    SPI1TxDMA_Handler.Init.Direction = DMA_MEMORY_TO_PERIPH;           //���赽�洢��
    SPI1TxDMA_Handler.Init.PeriphInc = DMA_PINC_DISABLE;               //���������ģʽ
    SPI1TxDMA_Handler.Init.MemInc    = DMA_MINC_ENABLE;                //�洢������ģʽ
    SPI1TxDMA_Handler.Init.PeriphDataAlignment=DMA_MDATAALIGN_HALFWORD;	   //�������ݳ���:8λ
    SPI1TxDMA_Handler.Init.MemDataAlignment=DMA_MDATAALIGN_HALFWORD;   //�洢�����ݳ���:8λ
    SPI1TxDMA_Handler.Init.Mode=DMA_NORMAL;                            //��������ģʽ
    SPI1TxDMA_Handler.Init.Priority=DMA_PRIORITY_MEDIUM;               //�е����ȼ�
    SPI1TxDMA_Handler.Init.FIFOMode=DMA_FIFOMODE_DISABLE;              

    HAL_DMA_DeInit(&SPI1TxDMA_Handler);   
    HAL_DMA_Init(&SPI1TxDMA_Handler);
	
		HAL_NVIC_SetPriority( DMA2_Stream2_IRQn, 1, 1 );  //����DMA�ж����ȼ�
		HAL_NVIC_EnableIRQ( DMA2_Stream2_IRQn );

		HAL_NVIC_SetPriority( DMA2_Stream5_IRQn, 1, 1 );  //����DMA�ж����ȼ�
		HAL_NVIC_EnableIRQ( DMA2_Stream5_IRQn );

}

void SPI1_Init(void)
{
    SPI1_Handler.Instance=SPI1;                         								//SPI1
    SPI1_Handler.Init.Mode=SPI_MODE_MASTER;             								//����SPI����ģʽ������Ϊ��ģʽ
    SPI1_Handler.Init.Direction=SPI_DIRECTION_2LINES;   								//����SPI�������˫�������ģʽ:SPI����Ϊ˫��ģʽ
    SPI1_Handler.Init.DataSize=SPI_DATASIZE_16BIT;       								//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
    SPI1_Handler.Init.CLKPolarity=SPI_POLARITY_LOW;    									//����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
    SPI1_Handler.Init.CLKPhase=SPI_PHASE_2EDGE;         								//����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
    SPI1_Handler.Init.NSS=SPI_NSS_SOFT;                 								//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
    SPI1_Handler.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_32;//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
    SPI1_Handler.Init.FirstBit=SPI_FIRSTBIT_MSB;        								//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
    SPI1_Handler.Init.TIMode=SPI_TIMODE_DISABLE;        //�ر�TIģʽ
    SPI1_Handler.Init.CRCCalculation=SPI_CRCCALCULATION_DISABLE;//�ر�Ӳ��CRCУ��
    SPI1_Handler.Init.CRCPolynomial=7;                  //CRCֵ����Ķ���ʽ
    HAL_SPI_Init(&SPI1_Handler);//��ʼ��
	
    __HAL_SPI_ENABLE(&SPI1_Handler);                    //ʹ��SPI1
		
		SPI1_DMA_config();

}

//�˺����ᱻHAL_SPI_Init()����
//hspi:SPI���
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_GPIOA_CLK_ENABLE();       //ʹ��GPIOBʱ��
    __HAL_RCC_SPI1_CLK_ENABLE();        //ʹ��SPI1ʱ��
    
    //PBA,5,6,7
    GPIO_Initure.Pin=GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;              //�����������
    GPIO_Initure.Pull=GPIO_PULLUP;                  //����
    GPIO_Initure.Speed=GPIO_SPEED_FAST;             //����            
    GPIO_Initure.Alternate=GPIO_AF5_SPI1;           //����ΪSPI1
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
}

//SPI�ٶ����ú���
//SPI�ٶ�=fAPB1/��Ƶϵ��
//@ref SPI_BaudRate_Prescaler:SPI_BAUDRATEPRESCALER_2~SPI_BAUDRATEPRESCALER_2 256
//fAPB1ʱ��һ��Ϊ42Mhz��
void SPI1_SetSpeed(u8 SPI_BaudRatePrescaler)
{
    assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));//�ж���Ч��
    __HAL_SPI_DISABLE(&SPI1_Handler);            //�ر�SPI
    SPI1_Handler.Instance->CR1&=0XFFC7;          //λ3-5���㣬�������ò�����
    SPI1_Handler.Instance->CR1|=SPI_BaudRatePrescaler;//����SPI�ٶ�
    __HAL_SPI_ENABLE(&SPI1_Handler);             //ʹ��SPI
    
}

//SPI1 ��д2���ֽ�
//TxData:Ҫд����ֽ�
//����ֵ:��ȡ�����ֽ�
uint16_t SPI1_ReadWriteByte(uint16_t TxData)
{
    uint16_t Rxdata;
    HAL_SPI_TransmitReceive(&SPI1_Handler,(uint8_t*)&TxData,(uint8_t*)&Rxdata,1, 1000);       
 	return Rxdata;          		    //�����յ�������		
}



//��Ҫһֱ���ͻ��߽��վ��ڻص����ٵ���һ�ν��ջ��ȡ����
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
//	HAL_SPI_Receive_DMA(&hspi, recv_buf, 5);				//������һֱ���գ�������Ҫע�ⲻ�ܵ���HAL_SPI_STOP�������������������ģʽ�����õġ�
	
}

void DMA2_Stream2_IRQHandler(void)			//���빫���жϺ����Ժ�HAL����Զ�������жϱ�־λ������Ҫ���ǲ��ģ������Ҫ���ж�֮�����£���ֱ���ڻص�������д�Ϳ���
{

		HAL_DMA_IRQHandler(&SPI1RxDMA_Handler);
}

void DMA2_Stream5_IRQHandler(void)		//���﷢�ͺͽ���Ҫͬʱ���ã������жϱ�־λ�޷�����ɾ�
{
	
		HAL_DMA_IRQHandler(&SPI1TxDMA_Handler);
}





