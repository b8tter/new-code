/*
*.............................
*write by xujunhui
*https://github.com/Kirinnana
*test zhouwentao
*hardware zhangjun
*/
#include "system.h"
#include "usart3.h"


#ifdef CAMERA 

#define USART_MAX_LEN 13
#define DMA_USART3_TX_BUF_LEN 13
volatile uint16_t com3_rx_len = 0;		 // 接收帧数据的长度
volatile uint8_t com3_recv_end_flag = 0; // 帧数据接收完成标志
uint8_t DMA_USART3_RX_BUF[14] = {0};		 // 接收数据缓存
uint8_t DMA_USART3_TX_BUF[8] = {0};		 // DMA发送缓存

__attribute__((noinline)) void USART3_Config(void)
{
	// GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);  // 使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); // 使能USART3时钟

	// 串口3对应引脚复用映射
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3); // GPIOB11复用为USART3 Rx
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3); // GPIOB10复用为USART3 TX

	// USART3端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_10; // GPIOB与GPIOB10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			 // 复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 // 速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			 // 推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			 // 上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 // 初始化PB11，PB10

	// USART3 初始化设置
	USART_InitStructure.USART_BaudRate = 115200;									// 波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						// 字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							// 一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;								// 无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// 收发模式
	USART_Init(USART3, &USART_InitStructure);										// 初始化串口3

/*
	USART_Cmd(USART3, ENABLE); // 使能串口1

	USART_ClearFlag(USART3, USART_FLAG_TC);

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); // 开启相关中断

	// Usart3 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;		  // 串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // 抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		  // 子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);							  // 根据指定的参数初始化VIC寄存器、

*/

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;		  // 串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; // 抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		  // 子优先级1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);							  // 根据指定的参数初始化VIC寄存器、

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream1_IRQn;	  // 嵌套通道为DMA1_Stream1_IRQn
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; // 抢占优先级为 2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  // 响应优先级为 2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // 通道中断使能
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;	  // 串口2发送中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; // 抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		  // 子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);

	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE); // 开启串口空闲中断
	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE); // 开启串口DMA接收
	USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE); // 开启串口DMA接收
												   /* 配置串口DMA接收*/

	DMAx_Init(DMA1_Stream1,DMA_Channel_4,(uint32_t)&USART3->DR,(uint32_t)DMA_USART3_RX_BUF,USART_MAX_LEN,DMA_DIR_PeripheralToMemory);

	// RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE); // 开启DMA时钟
	// DMA_DeInit(DMA1_Stream1);
	// DMA_InitStructure.DMA_Channel = DMA_Channel_4;							// 通道选择
	// DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;		/// DMA外设地址
	// DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)DMA_USART3_RX_BUF;		// DMA 存储器0地址
	// DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;					// 存储器到外设模式
	// DMA_InitStructure.DMA_BufferSize = USART_MAX_LEN;						// 数据传输量
	// DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		// 外设非增量模式
	// DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					// 存储器增量模式
	// DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 外设数据长度:8位
	// DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			// 存储器数据长度:8位
	// DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;							// 使用普通模式
	// DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;					// 高等优先级
	// DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;					// 不开启FIFO模式
	// DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;			// FIFO阈值
	// DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;				// 存储器突发单次传输
	// DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;		// 外设突发单次传输
	// DMA_Init(DMA1_Stream1, &DMA_InitStructure);
	// DMA_Cmd(DMA1_Stream1, ENABLE); // 使能DMA2_Stream5通道

	DMAx_Init(DMA1_Stream3,DMA_Channel_4,(uint32_t)&USART3->DR,(u32)DMA_USART3_TX_BUF,DMA_USART3_TX_BUF_LEN,DMA_DIR_MemoryToPeripheral);

	// DMA_DeInit(DMA1_Stream3); // 初始化DMA Stream
	// while (DMA_GetCmdStatus(DMA1_Stream3) != DISABLE)
	// 	; // 等待DMA可配置
	// /* 配置DMA1 Stream6，USART3发送 */
	// DMA_InitStructure.DMA_Channel = DMA_Channel_4;							// 通道选择
	// DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART3->DR;			// DMA外设地址
	// DMA_InitStructure.DMA_Memory0BaseAddr = (u32)DMA_USART3_TX_BUF;			// DMA 存储器0地
	// DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;					// 存储器到外设模式
	// DMA_InitStructure.DMA_BufferSize = DMA_USART3_TX_BUF_LEN;				// 数据传输量
	// DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		// 外设非增量模式
	// DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					// 存储器增量模式
	// DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 外设数据长度:8位
	// DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			// 存储器数据长度:8
	// DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;							// 使用普通模式
	// DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;					// 中等优先级
	// DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	// DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	// DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single; // 存储器突发单次传输

	// DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; // 外设突发单次传输

	// DMA_Init(DMA1_Stream3, &DMA_InitStructure); // 初始化DMA Stream6

	DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE); // DMA1传输完成中断
	DMA_Cmd(DMA1_Stream3, DISABLE);				   // 不使能
	USART_Cmd(USART3, ENABLE);					   // 使能串口3
}

// void USART3_IRQHandler(void) // 串口3中断服务程序
// {
// 	uint8_t temp;
// 	if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) // 空闲中断触发
// 	{
// 		temp = USART_ReceiveData(USART3);
// 		// Usart_SendByte(USART3, temp);
// 		Usart_SendByte(USART1, temp);
// 	}
// }


// float line_dif_temp;
//A5 5A 06 03 00 00 00 00 00 08 
void USART3_IRQHandler(void) // 串口3中断服务程序
{
  static u8 error_flag = 0;
  if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET) // 空闲中断触发
  {

    DMA_Cmd(DMA1_Stream1, DISABLE); /* 暂时关闭dma，数据尚未处理 */

    com3_rx_len = USART_MAX_LEN - DMA_GetCurrDataCounter(DMA1_Stream1); /* 获取接收到的数据长度 单位为字节*/

    DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1); /* 清DMA标志位 */
                                                 // DMA_SetCurrDataCounter(DMA1_Stream1,USART_MAX_LEN);/* 重新赋值计数值，必须大于等于最大可能接收到的数据帧数目 */
										 
    if (DMA_USART3_RX_BUF[0] == 0xa5 && DMA_USART3_RX_BUF[1] == 0x5a && DMA_USART3_RX_BUF[2] == 0x06)
    {
      if (com3_rx_len >= USART_MAX_LEN - 1) // 数据接收完毕
      {
		// Usart_SendByte(USART3,0x66);
		if(CHeck(&DMA_USART3_RX_BUF[0],com3_rx_len - 1) == DMA_USART3_RX_BUF[com3_rx_len - 1])
        
		{
			line_de_mode = DMA_USART3_RX_BUF[3];
			memcpy(&line_dif, &DMA_USART3_RX_BUF[4], 4);
			is_node = DMA_USART3_RX_BUF[8];
			is_find_line = DMA_USART3_RX_BUF[9];
			door_color = DMA_USART3_RX_BUF[10];
			// line_dif = line_dif_temp;
			com3_recv_end_flag = 1; // 接受完成标志位置1
			// printf("%d\n",door_color);
		}
		// else
		// {
		// 	printf("+++");
		// 	Usart_SendArray(USART1,DMA_USART3_RX_BUF,12);
		// }
		com3_rx_len = 0; // 清缓存
      }
	  memset(DMA_USART3_RX_BUF, 0, LENARRAY(DMA_USART3_RX_BUF));
      DMA_SetCurrDataCounter(DMA1_Stream1, USART_MAX_LEN); // 重新设置传输的数据数量

      DMA_Cmd(DMA1_Stream1, ENABLE); // 开启DMA传输
    }
	    else
    {

      if (error_flag)//接收错误
      {
        // Usart_SendArray(USART1,com1_rx_buffer,25);
        error_flag = 0;
		memset(DMA_USART3_RX_BUF, 0, LENARRAY(DMA_USART3_RX_BUF));
        DMA_SetCurrDataCounter(DMA1_Stream1, USART_MAX_LEN); // 重新设置传输的数据数量
        DMA_Cmd(DMA1_Stream1, ENABLE);                       // 开启DMA传输
      }
      else
      {
		static short int i;
        for (i = 1; i <= USART_MAX_LEN - 1; i++)
        {
          if (DMA_USART3_RX_BUF[i] == 0xa5) // 寻找帧头位置，接收多余数据
          {
            memset(DMA_USART3_RX_BUF, 0, LENARRAY(DMA_USART3_RX_BUF));
            DMA_SetCurrDataCounter(DMA1_Stream1, i); // 重新设置传输的数据数量
            DMA_Cmd(DMA1_Stream1, ENABLE);                           // 开启DMA传输
            error_flag = 1;
            break;
          }
        }
		if(!error_flag)
		{
		memset(DMA_USART3_RX_BUF, 0, LENARRAY(DMA_USART3_RX_BUF));
      	DMA_SetCurrDataCounter(DMA1_Stream1, USART_MAX_LEN); // 重新设置传输的数据数量
      	DMA_Cmd(DMA1_Stream1, ENABLE); // 开启DMA传输
		}
      }
    }
	USART_ReceiveData(USART3); // 清除空闲中断标志位（接收函数有清标志位的作用）
  }
	//printf("%\n%d\n",error_flag);


  if (USART_GetFlagStatus(USART3, USART_IT_TXE) == RESET) // 串口发送完成
  {
    USART_ITConfig(USART3, USART_IT_TC, DISABLE);
  }
}

__attribute__((noinline)) void Change_mode(u8 mode, short int dealy_time) // 切换摄像头检测模式
{
	static long int last_second = 0;
	static short int last_millisencond = 0;
	static short int is_init = 1, send_flag;
	if (is_init)
	{
		is_init = 0;
		last_millisencond = millisencond_count;
		last_second = second_count;
		send_flag = 1;
	}
	else if (time_difference(last_second, last_millisencond, second_count, millisencond_count, 1000) >= dealy_time)
	{
		last_millisencond = millisencond_count;
		last_second = second_count;
		send_flag = 1;
	}
	if (send_flag)
	{
		DMA_USART3_TX_BUF[0] = 0xa5;
		DMA_USART3_TX_BUF[1] = 0x5a;
		DMA_USART3_TX_BUF[2] = 0x06;
		switch (mode)
		{
		case 1: // 检测平台
			DMA_USART3_TX_BUF[3] = 0x01;
			DMA_USART3_TX_BUF[4] = 0x06;


			break;
		case 2: // 查找节点
			DMA_USART3_TX_BUF[3] = 0x02;
			DMA_USART3_TX_BUF[4] = 0x07;


			break;
		case 3: // 正常巡线
			DMA_USART3_TX_BUF[3] = 0x03;
			DMA_USART3_TX_BUF[4] = 0x08;


			break;
		case 4: // 正常巡线
			DMA_USART3_TX_BUF[3] = 0x04;
			DMA_USART3_TX_BUF[4] = 0x09;
			

			break;			
		default:
			break;

		}
		DMA_USARTx_Send(DMA1_Stream3, 5);
		send_flag = 0;
	}
}

void DMA1_Stream3_IRQHandler(void)
{
	//清除标志
	if(DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TCIF3)!=RESET)//等待DMA传输完成
	{
		DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3); //清除DMA传输完成标志
   		DMA_Cmd(DMA1_Stream3,DISABLE);				//关闭使能
    	USART_ITConfig(USART3,USART_IT_TC,ENABLE);  //打开串口发送完成中断
	}
}
#endif

