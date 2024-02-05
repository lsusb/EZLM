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

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);//使能SPI1时钟

    //GPIOA5,6,7初始化设置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;//PB5~7复用功能输出
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化

    GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1); //PA5复用为 SPI1  CLK
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1); //PA6复用为 SPI1  MISO
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1); //PA7复用为 SPI1  MOSI
		


    //这里只针对SPI口初始化
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,ENABLE);//复位SPI1
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,DISABLE);//停止复位SPI1

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;		//设置SPI的数据大小:SPI发送接收16位帧结构
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//串行同步时钟的空闲状态为高电平
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//串行同步时钟的第二个跳变沿（上升或下降）数据被采样
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;		//定义波特率预分频的值:波特率预分频值为8
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
    SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
    SPI_Init(SPI1, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器

    SPI_Cmd(SPI1, ENABLE); //使能SPI外设
		
		SPI1_TX_DMA_Config();
		
		SPI1_RX_DMA_Config();
		
		
		//SPI2 TX DMA请求使能
		SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);	
		//SPI2 RX DMA请求使能
		SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);	

		

//    SPI1_ReadWriteByte(0xff);//启动传输
}


#define SENDBUFF_SIZE (1)	    // 一次发送的数据	
uint16_t TX_Buff[2];		// 发送缓存
void SPI1_TX_DMA_Config(void)
{
    // 中断结构体
    NVIC_InitTypeDef NVIC_InitStructure;		
    // DMA结构体
    DMA_InitTypeDef DMA_InitStructure;  		
    /* 使能DMA时钟 */  		
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);	
    /* 复位初始化DMA数据流 */  
    DMA_DeInit(DMA2_Stream5);								
    /* 确保DMA数据流复位完成 */  
    while (DMA_GetCmdStatus(DMA2_Stream5) != DISABLE);	

    /* 配置 DMA Stream */
    /* 通道3，数据流5 */	  
    DMA_InitStructure.DMA_Channel = DMA_Channel_3;
    /* 外设地址 */  
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&SPI1->DR;	
    /* 内存地址(要传输的变量的指针) ,DMA存储器0地址*/  	
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)TX_Buff;	
    /* 方向：存储器到外设 */			
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    /* 数据传输量 ,可设置为0， 实际发送时会重新设置*/	    
    DMA_InitStructure.DMA_BufferSize = (uint32_t)SENDBUFF_SIZE;		
    /* 外设非增量模式 */		
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    /* 存储器增量模式 */  	
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    /* 外设数据长度:16位 */	 
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    /* 内存数据长度:16位 */
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    /* DMA模式：正常模式 */  		
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    /* 优先级：高 */	 		
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    /* 禁用FIFO */
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;        		
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;   
    /* 外设突发单次传输 */  
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;    		
    /* 存储器突发单次传输 */  
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; 

    /* 初始化DMA Stream */		
    DMA_Init(DMA2_Stream5, &DMA_InitStructure);
    /* 开启传输完成中断  */		
    DMA_ITConfig(DMA2_Stream5,DMA_IT_TC,ENABLE);

    // 中断初始化 
    /* DMA发送中断源 */  
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream5_IRQn;	
    /* 抢断优先级 */  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    /* 响应优先级 */  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;				
    /* 使能外部中断通道 */ 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;						 
    /* 配置NVIC */		
    NVIC_Init(&NVIC_InitStructure);
}

#define RECEIVE_SIZE  		1  	// 接收大小
uint16_t RX_Buff[2];		// 接收到缓存
void SPI1_RX_DMA_Config(void)
{
    // 中断结构体
    NVIC_InitTypeDef NVIC_InitStructure;	
    // DMA结构体  
    DMA_InitTypeDef DMA_InitStructure;		
    /* 使能DMA时钟*/  
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);					/* 复位初始化DMA数据流 */ 
    DMA_DeInit(DMA2_Stream2);												/* 确保DMA数据流复位完成 */
    while(DMA_GetCmdStatus(DMA2_Stream2)!=DISABLE);

    /* 配置 DMA Stream */
    /* 通道3，数据流2*/	  
    DMA_InitStructure.DMA_Channel = DMA_Channel_3;  			
    /* 设置DMA源：串口数据寄存器地址*/  
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&SPI1->DR		;
    /* 内存地址(要传输的变量的指针)*/  
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)RX_Buff;			
    /* 方向：存储器到外设模式 */			
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    /* 数据传输量 ,需要最大可能接受的数据量[不能为0],实际发送时会重新设置*/	  
    DMA_InitStructure.DMA_BufferSize = (uint32_t)RECEIVE_SIZE;
    /* 外设非增量模式 */	  
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 	  
    /* 存储器增量模式 */    
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    /* 外设数据长度:16位 */	  
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    /* 内存数据长度16位 */
    DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_HalfWord;		
    /* DMA模式：正常模式 */  
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    /* 优先级：高 */	   
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    /*禁用FIFO*/    
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    /* 外设突发单次传输 */  
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    /* 存储器突发单次传输 */  
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; 
    /* 初始化DMA Stream */		
    DMA_Init(DMA2_Stream2, &DMA_InitStructure);							   
    /* 开启传输完成中断  */
    DMA_ITConfig(DMA2_Stream2,DMA_IT_TC,ENABLE);  					

    // 中断初始化 
    /* 配置 DMA接收为中断源 */
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream2_IRQn;  	
    /* 抢断优先级 */  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;		
    /* 响应优先级 */  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;				
    /* 使能外部中断通道 */  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    /* 配置NVIC */	
    NVIC_Init(&NVIC_InitStructure);
		
		DMA_Cmd(DMA2_Stream2, ENABLE);

}







//SPI1速度设置函数
//SPI速度=fAPB2/分频系数
//@ref SPI_BaudRate_Prescaler:SPI_BaudRatePrescaler_2~SPI_BaudRatePrescaler_256
//fAPB2时钟一般为84Mhz：
void SPI1_SetSpeed(u8 SPI_BaudRatePrescaler)
{
    assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));//判断有效性
    SPI1->CR1&=0XFFC7;//位3-5清零，用来设置波特率
    SPI1->CR1|=SPI_BaudRatePrescaler;	//设置SPI1速度
    SPI_Cmd(SPI1,ENABLE); //使能SPI1
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

//SPI_DMA 读写一个buf
#define BufSize		1
void SPI_DMA_WRITE_READ_BUF(void) 
{   
		TX_Buff[0] = AS5048_ANGLE;
		
    // 关闭发送 DMA	
    DMA_Cmd(DMA2_Stream5, DISABLE);	
    // 关闭接收 DMA	
    DMA_Cmd(DMA2_Stream2, DISABLE);
    // 设置发送的数据量
    DMA_SetCurrDataCounter(DMA2_Stream5, BufSize);	
    // 设置接收的数据量
    DMA_SetCurrDataCounter(DMA2_Stream2, BufSize);
	// 清空数据
    SPI1->DR;			  	
    // 擦除DMA标志位
    DMA_ClearFlag(DMA2_Stream5, DMA_IT_TCIF5);	
    DMA_ClearFlag(DMA2_Stream2, DMA_IT_TCIF2);

    // 片选拉低,接收数据
    AS5048_LOW;	
    // 开启接收 DMA
    DMA_Cmd(DMA2_Stream5, ENABLE);	
    DMA_Cmd(DMA2_Stream2, ENABLE);
}




//DMA发送中断服务函数
void DMA2_Stream5_IRQHandler(void)
{
    // DMA 发送完成
    if(DMA_GetITStatus(DMA2_Stream5, DMA_IT_TCIF5))	
    {
        // 清除DMA发送完成标志
        DMA_ClearITPendingBit(DMA2_Stream5, DMA_IT_TCIF5);	
//        // 片选拉高，数据发送完毕	
//        AS5048_HIGH;	
    }
}



//DMA接收中断服务函数
uint16_t roter_pos_conpensation = 45;    //410

void DMA2_Stream2_IRQHandler(void)
{				
		static uint16_t as5048_reg_last = 0;
	
	
	
    // DMA接收完成
    if(DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2))	
    {		
        // 数据接收完成 拉高片选
        AS5048_HIGH;	
        // 清除DMA接收完成标志位		
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
				
				senser_as5048.ElectricAngle = ((senser_as5048.reg_cal*11 % 16384) - 8192) * 4;  //机械角度换电角度
				
//				senser_as5048.ElectricAngle = (senser_as5048.reg_cal*11 % 16384)/16384.0f *360.0f;  //机械角度换电角度
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
		
	
	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOC时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);//使能SPI3时钟


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;//PC10,11,12复用功能输出
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化

    GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_SPI3); //PA5复用为 SPI1  CLK
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_SPI3); //PA6复用为 SPI1  MISO
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_SPI3); //PA7复用为 SPI1  MOSI


    //这里只针对SPI口初始化
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3,ENABLE);//复位SPI3
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3,DISABLE);//停止复位SPI1

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;		//设置SPI的数据大小:SPI发送接收16位帧结构
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//串行同步时钟的空闲状态为高电平
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//串行同步时钟的第二个跳变沿（上升或下降）数据被采样
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;		//定义波特率预分频的值:波特率预分频值为8
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
    SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
    SPI_Init(SPI3, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器

    SPI_Cmd(SPI3, ENABLE); //使能SPI外设	
	
}



uint16_t SPI3_ReadWrite16Bit(uint16_t Txdata)
{
    while(SPI_I2S_GetFlagStatus(SPI3,SPI_I2S_FLAG_TXE) == RESET) {}
    SPI_I2S_SendData(SPI3,Txdata);
    while(SPI_I2S_GetFlagStatus(SPI3,SPI_I2S_FLAG_RXNE) == RESET) {}
    return SPI_I2S_ReceiveData(SPI3);
}



