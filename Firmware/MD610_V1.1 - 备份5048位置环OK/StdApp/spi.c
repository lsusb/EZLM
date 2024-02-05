#include "spi.h"

#include "stm32f4xx_spi.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

#include "as5048.h"
#include "control.h"


#include "mc_svpwm.h"



void SPI1_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��GPIOAʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);//ʹ��SPI1ʱ��

    //GPIOA5,6,7��ʼ������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;//PB5~7���ù������
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��

    GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1); //PA5����Ϊ SPI1  CLK
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1); //PA6����Ϊ SPI1  MISO
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1); //PA7����Ϊ SPI1  MOSI
		


    //����ֻ���SPI�ڳ�ʼ��
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,ENABLE);//��λSPI1
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,DISABLE);//ֹͣ��λSPI1

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//����SPI����ģʽ:����Ϊ��SPI
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;		//����SPI�����ݴ�С:SPI���ͽ���16λ֡�ṹ
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ8
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
    SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRCֵ����Ķ���ʽ
    SPI_Init(SPI1, &SPI_InitStructure);  //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���

    SPI_Cmd(SPI1, ENABLE); //ʹ��SPI����
		
		SPI1_TX_DMA_Config();
		
		SPI1_RX_DMA_Config();
		
		
		//SPI2 TX DMA����ʹ��
		SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);	
		//SPI2 RX DMA����ʹ��
		SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);	

		

//    SPI1_ReadWriteByte(0xff);//��������
}


#define SENDBUFF_SIZE (1)	    // һ�η��͵�����	
uint16_t TX_Buff[2];		// ���ͻ���
void SPI1_TX_DMA_Config(void)
{
    // �жϽṹ��
    NVIC_InitTypeDef NVIC_InitStructure;		
    // DMA�ṹ��
    DMA_InitTypeDef DMA_InitStructure;  		
    /* ʹ��DMAʱ�� */  		
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);	
    /* ��λ��ʼ��DMA������ */  
    DMA_DeInit(DMA2_Stream5);								
    /* ȷ��DMA��������λ��� */  
    while (DMA_GetCmdStatus(DMA2_Stream5) != DISABLE);	

    /* ���� DMA Stream */
    /* ͨ��3��������5 */	  
    DMA_InitStructure.DMA_Channel = DMA_Channel_3;
    /* �����ַ */  
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&SPI1->DR;	
    /* �ڴ��ַ(Ҫ����ı�����ָ��) ,DMA�洢��0��ַ*/  	
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)TX_Buff;	
    /* ���򣺴洢�������� */			
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    /* ���ݴ����� ,������Ϊ0�� ʵ�ʷ���ʱ����������*/	    
    DMA_InitStructure.DMA_BufferSize = (uint32_t)SENDBUFF_SIZE;		
    /* ���������ģʽ */		
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    /* �洢������ģʽ */  	
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    /* �������ݳ���:16λ */	 
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    /* �ڴ����ݳ���:16λ */
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    /* DMAģʽ������ģʽ */  		
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    /* ���ȼ����� */	 		
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    /* ����FIFO */
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;        		
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;   
    /* ����ͻ�����δ��� */  
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;    		
    /* �洢��ͻ�����δ��� */  
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; 

    /* ��ʼ��DMA Stream */		
    DMA_Init(DMA2_Stream5, &DMA_InitStructure);
    /* ������������ж�  */		
    DMA_ITConfig(DMA2_Stream5,DMA_IT_TC,ENABLE);

    // �жϳ�ʼ�� 
    /* DMA�����ж�Դ */  
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream5_IRQn;	
    /* �������ȼ� */  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    /* ��Ӧ���ȼ� */  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;				
    /* ʹ���ⲿ�ж�ͨ�� */ 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;						 
    /* ����NVIC */		
    NVIC_Init(&NVIC_InitStructure);
}

#define RECEIVE_SIZE  		1  	// ���մ�С
uint16_t RX_Buff[2];		// ���յ�����
void SPI1_RX_DMA_Config(void)
{
    // �жϽṹ��
    NVIC_InitTypeDef NVIC_InitStructure;	
    // DMA�ṹ��  
    DMA_InitTypeDef DMA_InitStructure;		
    /* ʹ��DMAʱ��*/  
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);					/* ��λ��ʼ��DMA������ */ 
    DMA_DeInit(DMA2_Stream2);												/* ȷ��DMA��������λ��� */
    while(DMA_GetCmdStatus(DMA2_Stream2)!=DISABLE);

    /* ���� DMA Stream */
    /* ͨ��3��������2*/	  
    DMA_InitStructure.DMA_Channel = DMA_Channel_3;  			
    /* ����DMAԴ���������ݼĴ�����ַ*/  
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&SPI1->DR		;
    /* �ڴ��ַ(Ҫ����ı�����ָ��)*/  
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)RX_Buff;			
    /* ���򣺴洢��������ģʽ */			
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    /* ���ݴ����� ,��Ҫ�����ܽ��ܵ�������[����Ϊ0],ʵ�ʷ���ʱ����������*/	  
    DMA_InitStructure.DMA_BufferSize = (uint32_t)RECEIVE_SIZE;
    /* ���������ģʽ */	  
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 	  
    /* �洢������ģʽ */    
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    /* �������ݳ���:16λ */	  
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    /* �ڴ����ݳ���16λ */
    DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_HalfWord;		
    /* DMAģʽ������ģʽ */  
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    /* ���ȼ����� */	   
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    /*����FIFO*/    
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    /* ����ͻ�����δ��� */  
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    /* �洢��ͻ�����δ��� */  
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; 
    /* ��ʼ��DMA Stream */		
    DMA_Init(DMA2_Stream2, &DMA_InitStructure);							   
    /* ������������ж�  */
    DMA_ITConfig(DMA2_Stream2,DMA_IT_TC,ENABLE);  					

    // �жϳ�ʼ�� 
    /* ���� DMA����Ϊ�ж�Դ */
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream2_IRQn;  	
    /* �������ȼ� */  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;		
    /* ��Ӧ���ȼ� */  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;				
    /* ʹ���ⲿ�ж�ͨ�� */  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    /* ����NVIC */	
    NVIC_Init(&NVIC_InitStructure);
		
		DMA_Cmd(DMA2_Stream2, ENABLE);

}







//SPI1�ٶ����ú���
//SPI�ٶ�=fAPB2/��Ƶϵ��
//@ref SPI_BaudRate_Prescaler:SPI_BaudRatePrescaler_2~SPI_BaudRatePrescaler_256
//fAPB2ʱ��һ��Ϊ84Mhz��
void SPI1_SetSpeed(u8 SPI_BaudRatePrescaler)
{
    assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));//�ж���Ч��
    SPI1->CR1&=0XFFC7;//λ3-5���㣬�������ò�����
    SPI1->CR1|=SPI_BaudRatePrescaler;	//����SPI1�ٶ�
    SPI_Cmd(SPI1,ENABLE); //ʹ��SPI1
}


uint16_t SPI1_ReadWrite16Bit(uint16_t Txdata)
{
    while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE) == RESET) {}
    SPI_I2S_SendData(SPI1,Txdata);
    while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE) == RESET) {}
    return SPI_I2S_ReceiveData(SPI1);
}


void Spi1_Function(int OutputBuff[],int16_t InputBuff[], unsigned int NoOfBytes)
{
    int i;

    for(i=0; i<(NoOfBytes); i++)
    {
        InputBuff[i]=SPI1_ReadWrite16Bit( OutputBuff[i]);
    }

}

//SPI_DMA ��дһ��buf
#define BufSize		1
void SPI_DMA_WRITE_READ_BUF(void) 
{   
		TX_Buff[0] = AS5048_ANGLE;
		
    // �رշ��� DMA	
    DMA_Cmd(DMA2_Stream5, DISABLE);	
    // �رս��� DMA	
    DMA_Cmd(DMA2_Stream2, DISABLE);
    // ���÷��͵�������
    DMA_SetCurrDataCounter(DMA2_Stream5, BufSize);	
    // ���ý��յ�������
    DMA_SetCurrDataCounter(DMA2_Stream2, BufSize);
	// �������
    SPI1->DR;			  	
    // ����DMA��־λ
    DMA_ClearFlag(DMA2_Stream5, DMA_IT_TCIF5);	
    DMA_ClearFlag(DMA2_Stream2, DMA_IT_TCIF2);

    // Ƭѡ����,��������
    AS5048_LOW;	
    // �������� DMA
    DMA_Cmd(DMA2_Stream5, ENABLE);	
    DMA_Cmd(DMA2_Stream2, ENABLE);
}




//DMA�����жϷ�����
void DMA2_Stream5_IRQHandler(void)
{
    // DMA �������
    if(DMA_GetITStatus(DMA2_Stream5, DMA_IT_TCIF5))	
    {
        // ���DMA������ɱ�־
        DMA_ClearITPendingBit(DMA2_Stream5, DMA_IT_TCIF5);	
//        // Ƭѡ���ߣ����ݷ������	
//        AS5048_HIGH;	
    }
}



//DMA�����жϷ�����
uint16_t roter_pos_conpensation = 45;    //410

void DMA2_Stream2_IRQHandler(void)
{				
		static uint16_t as5048_reg_last = 0;
	
	
	
    // DMA�������
    if(DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2))	
    {		
        // ���ݽ������ ����Ƭѡ
        AS5048_HIGH;	
        // ���DMA������ɱ�־λ		
        DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);	
			
				senser_as5048.reg = RX_Buff[0]&0x3fff;
			
				if(as5048_reg_last - senser_as5048.reg > 16384/2)
						senser_as5048.cnt ++;
				if(as5048_reg_last - senser_as5048.reg < -16384/2)
						senser_as5048.cnt --;
				senser_as5048.pos = senser_as5048.reg + senser_as5048.cnt * 16384;
				 
				
				if(senser_as5048.reg >= roter_pos_conpensation)
				{
						senser_as5048.reg_cal = senser_as5048.reg - roter_pos_conpensation;
				}else
				{
						senser_as5048.reg_cal = 16384 - roter_pos_conpensation + senser_as5048.reg;
				}
				
				senser_as5048.ElectricAngle = ((senser_as5048.reg_cal*11 % 16384) - 8192) * 4;  //��е�ǶȻ���Ƕ�
				
//				senser_as5048.ElectricAngle = (senser_as5048.reg_cal*11 % 16384)/16384.0f *360.0f;  //��е�ǶȻ���Ƕ�
//				
//				mc_svpm.Park.Theta = (uint16_t)(senser_as5048.ElectricAngle * 10.0f);
//			
//				mc_svpm.Park.Theta = LIMIT(mc_svpm.Park.Theta,0,3599);
			
				as5048_reg_last = senser_as5048.reg;
    }
}


void SPI3_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;
	
	
    RCC_AHB1PeriphClockCmd(RCC_SPI3_CS, ENABLE );
	
    GPIO_InitStructure.GPIO_Pin =  SPI3_CS_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(SPI3_CS_PORT,&GPIO_InitStructure);
		
		SPI3_CS_HIGH;
		
	
	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//ʹ��GPIOCʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);//ʹ��SPI3ʱ��


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;//PC10,11,12���ù������
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��

    GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_SPI3); //PA5����Ϊ SPI1  CLK
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_SPI3); //PA6����Ϊ SPI1  MISO
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_SPI3); //PA7����Ϊ SPI1  MOSI


    //����ֻ���SPI�ڳ�ʼ��
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3,ENABLE);//��λSPI3
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3,DISABLE);//ֹͣ��λSPI1

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//����SPI����ģʽ:����Ϊ��SPI
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;		//����SPI�����ݴ�С:SPI���ͽ���16λ֡�ṹ
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ8
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
    SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRCֵ����Ķ���ʽ
    SPI_Init(SPI3, &SPI_InitStructure);  //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���

    SPI_Cmd(SPI3, ENABLE); //ʹ��SPI����	
	
}



uint16_t SPI3_ReadWrite16Bit(uint16_t Txdata)
{
    while(SPI_I2S_GetFlagStatus(SPI3,SPI_I2S_FLAG_TXE) == RESET) {}
    SPI_I2S_SendData(SPI3,Txdata);
    while(SPI_I2S_GetFlagStatus(SPI3,SPI_I2S_FLAG_RXNE) == RESET) {}
    return SPI_I2S_ReceiveData(SPI3);
}



