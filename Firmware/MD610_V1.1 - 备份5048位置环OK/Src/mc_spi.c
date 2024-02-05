#include "mc_spi.h"


SPI_HandleTypeDef SPI1_Handler;  //SPI1句柄
DMA_HandleTypeDef SPI1RxDMA_Handler;
DMA_HandleTypeDef SPI1TxDMA_Handler;

//配置DMA为单次模式
void SPI1_DMA_config(void)
{
		__HAL_RCC_DMA2_CLK_ENABLE();//DMA2时钟使能 
    __HAL_LINKDMA(&SPI1_Handler,hdmarx,SPI1RxDMA_Handler);              //将DMA与SPI2联系起来(发送DMA)
	
    //Rx DMA配置
    SPI1RxDMA_Handler.Instance       = DMA2_Stream2;                   		//数据流选择
    SPI1RxDMA_Handler.Init.Channel   = DMA_CHANNEL_3;                  		//通道选择
    SPI1RxDMA_Handler.Init.Direction = DMA_PERIPH_TO_MEMORY;           		//外设到存储器
    SPI1RxDMA_Handler.Init.PeriphInc = DMA_PINC_DISABLE;               		//外设非增量模式
    SPI1RxDMA_Handler.Init.MemInc    = DMA_MINC_ENABLE;                		//存储器增量模式
    SPI1RxDMA_Handler.Init.PeriphDataAlignment=DMA_MDATAALIGN_HALFWORD;	  //外设数据长度:16位
    SPI1RxDMA_Handler.Init.MemDataAlignment=DMA_MDATAALIGN_HALFWORD;   		//存储器数据长度:16位
    SPI1RxDMA_Handler.Init.Mode=DMA_NORMAL;                            		//外设流控模式
    SPI1RxDMA_Handler.Init.Priority=DMA_PRIORITY_MEDIUM;               		//中等优先级
    SPI1RxDMA_Handler.Init.FIFOMode=DMA_FIFOMODE_DISABLE;              
	
    HAL_DMA_DeInit(&SPI1RxDMA_Handler);   
    HAL_DMA_Init(&SPI1RxDMA_Handler);

		__HAL_LINKDMA(&SPI1_Handler,hdmatx,SPI1TxDMA_Handler);              //将DMA与SPI2联系起来(发送DMA)
	
    //Tx DMA配置
    SPI1TxDMA_Handler.Instance       = DMA2_Stream5;                   //数据流选择
    SPI1TxDMA_Handler.Init.Channel   = DMA_CHANNEL_3;                  //通道选择
    SPI1TxDMA_Handler.Init.Direction = DMA_MEMORY_TO_PERIPH;           //外设到存储器
    SPI1TxDMA_Handler.Init.PeriphInc = DMA_PINC_DISABLE;               //外设非增量模式
    SPI1TxDMA_Handler.Init.MemInc    = DMA_MINC_ENABLE;                //存储器增量模式
    SPI1TxDMA_Handler.Init.PeriphDataAlignment=DMA_MDATAALIGN_HALFWORD;	   //外设数据长度:8位
    SPI1TxDMA_Handler.Init.MemDataAlignment=DMA_MDATAALIGN_HALFWORD;   //存储器数据长度:8位
    SPI1TxDMA_Handler.Init.Mode=DMA_NORMAL;                            //外设流控模式
    SPI1TxDMA_Handler.Init.Priority=DMA_PRIORITY_MEDIUM;               //中等优先级
    SPI1TxDMA_Handler.Init.FIFOMode=DMA_FIFOMODE_DISABLE;              

    HAL_DMA_DeInit(&SPI1TxDMA_Handler);   
    HAL_DMA_Init(&SPI1TxDMA_Handler);
	
		HAL_NVIC_SetPriority( DMA2_Stream2_IRQn, 1, 1 );  //接收DMA中断优先级
		HAL_NVIC_EnableIRQ( DMA2_Stream2_IRQn );

		HAL_NVIC_SetPriority( DMA2_Stream5_IRQn, 1, 1 );  //发送DMA中断优先级
		HAL_NVIC_EnableIRQ( DMA2_Stream5_IRQn );

}

void SPI1_Init(void)
{
    SPI1_Handler.Instance=SPI1;                         								//SPI1
    SPI1_Handler.Init.Mode=SPI_MODE_MASTER;             								//设置SPI工作模式，设置为主模式
    SPI1_Handler.Init.Direction=SPI_DIRECTION_2LINES;   								//设置SPI单向或者双向的数据模式:SPI设置为双线模式
    SPI1_Handler.Init.DataSize=SPI_DATASIZE_16BIT;       								//设置SPI的数据大小:SPI发送接收8位帧结构
    SPI1_Handler.Init.CLKPolarity=SPI_POLARITY_LOW;    									//串行同步时钟的空闲状态为高电平
    SPI1_Handler.Init.CLKPhase=SPI_PHASE_2EDGE;         								//串行同步时钟的第二个跳变沿（上升或下降）数据被采样
    SPI1_Handler.Init.NSS=SPI_NSS_SOFT;                 								//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
    SPI1_Handler.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_32;//定义波特率预分频的值:波特率预分频值为256
    SPI1_Handler.Init.FirstBit=SPI_FIRSTBIT_MSB;        								//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
    SPI1_Handler.Init.TIMode=SPI_TIMODE_DISABLE;        //关闭TI模式
    SPI1_Handler.Init.CRCCalculation=SPI_CRCCALCULATION_DISABLE;//关闭硬件CRC校验
    SPI1_Handler.Init.CRCPolynomial=7;                  //CRC值计算的多项式
    HAL_SPI_Init(&SPI1_Handler);//初始化
	
    __HAL_SPI_ENABLE(&SPI1_Handler);                    //使能SPI1
		
		SPI1_DMA_config();

}

//此函数会被HAL_SPI_Init()调用
//hspi:SPI句柄
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_GPIOA_CLK_ENABLE();       //使能GPIOB时钟
    __HAL_RCC_SPI1_CLK_ENABLE();        //使能SPI1时钟
    
    //PBA,5,6,7
    GPIO_Initure.Pin=GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;              //复用推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;                  //上拉
    GPIO_Initure.Speed=GPIO_SPEED_FAST;             //快速            
    GPIO_Initure.Alternate=GPIO_AF5_SPI1;           //复用为SPI1
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
}

//SPI速度设置函数
//SPI速度=fAPB1/分频系数
//@ref SPI_BaudRate_Prescaler:SPI_BAUDRATEPRESCALER_2~SPI_BAUDRATEPRESCALER_2 256
//fAPB1时钟一般为42Mhz：
void SPI1_SetSpeed(u8 SPI_BaudRatePrescaler)
{
    assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));//判断有效性
    __HAL_SPI_DISABLE(&SPI1_Handler);            //关闭SPI
    SPI1_Handler.Instance->CR1&=0XFFC7;          //位3-5清零，用来设置波特率
    SPI1_Handler.Instance->CR1|=SPI_BaudRatePrescaler;//设置SPI速度
    __HAL_SPI_ENABLE(&SPI1_Handler);             //使能SPI
    
}

//SPI1 读写2个字节
//TxData:要写入的字节
//返回值:读取到的字节
uint16_t SPI1_ReadWriteByte(uint16_t TxData)
{
    uint16_t Rxdata;
    HAL_SPI_TransmitReceive(&SPI1_Handler,(uint8_t*)&TxData,(uint8_t*)&Rxdata,1, 1000);       
 	return Rxdata;          		    //返回收到的数据		
}



//需要一直发送或者接收就在回调里再调用一次接收或读取函数
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
//	HAL_SPI_Receive_DMA(&hspi, recv_buf, 5);				//这样会一直接收，这里需要注意不能调用HAL_SPI_STOP（）函数，这个是连续模式才有用的。
	
}

void DMA2_Stream2_IRQHandler(void)			//进入公用中断函数以后HAL库会自动的清除中断标志位，不需要我们操心，如果需要在中断之中做事，就直接在回调函数中写就可以
{

		HAL_DMA_IRQHandler(&SPI1RxDMA_Handler);
}

void DMA2_Stream5_IRQHandler(void)		//这里发送和接收要同时配置，否则中断标志位无法清除干净
{
	
		HAL_DMA_IRQHandler(&SPI1TxDMA_Handler);
}





