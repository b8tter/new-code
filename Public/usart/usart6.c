/*
*.............................
*write by xujunhui
*https://github.com/Kirinnana
*test zhouwentao
*hardware zhangjun
*/
#include "system.h"
#include "usart6.h"



#ifdef TOF400F
#define DMA_USART6_RX_MAX_LEN 10
#define DMA_USART6_TX_BUF_LEN 9
volatile uint16_t com6_rx_len = 0;		 // 接收帧数据的长度
volatile uint8_t com6_recv_end_flag = 0; // 帧数据接收完成标志
uint8_t DMA_USART6_RX_BUF[9] = {0};		 // 接收数据缓存
uint8_t DMA_USART6_TX_BUF[9] = {0};		 // DMA发送缓存

__attribute__((noinline)) void USART6_Config(void)
{
	// GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	// DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);  // 使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE); // 使能USART6时钟

	// 串口6对应引脚复用映射
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource9, GPIO_AF_USART6); // GPIOG9复用为USART6 Rx
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource14, GPIO_AF_USART6); // GPIOG14复用为USART6 TX

	// USART6端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_9; // GPIOB与GPIOB10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			 // 复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 // 速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			 // 推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			 // 上拉
	GPIO_Init(GPIOG, &GPIO_InitStructure);					 // 初始化

	// USART6 初始化设置
	USART_InitStructure.USART_BaudRate = 115200;									// 波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						// 字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							// 一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;								// 无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// 收发模式
	USART_Init(USART6, &USART_InitStructure);										// 初始化串口6

/*
	USART_Cmd(USART6, ENABLE); // 使能串口1

	USART_ClearFlag(USART6, USART_FLAG_TC);

	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE); // 开启相关中断

	// Usart6 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;		  // 串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // 抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		  // 子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);							  // 根据指定的参数初始化VIC寄存器、

*/

	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;		  // 串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; // 抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		  // 子优先级1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);							  // 根据指定的参数初始化VIC寄存器、

	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;	  // 嵌套通道为DMA2_Stream1_IRQn
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; // 抢占优先级为 2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  // 响应优先级为 2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // 通道中断使能
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream6_IRQn;	  // 串口2发送中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; // 抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		  // 子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);

	USART_ITConfig(USART6, USART_IT_IDLE, ENABLE); // 开启串口空闲中断
	USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE); // 开启串口DMA接收
	USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE); // 开启串口DMA接收
												   /* 配置串口DMA接收*/

	DMAx_Init(DMA2_Stream1,DMA_Channel_5,(uint32_t)&USART6->DR,(uint32_t)DMA_USART6_RX_BUF,DMA_USART6_RX_MAX_LEN,DMA_DIR_PeripheralToMemory);

	// RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE); // 开启DMA时钟
	// DMA_DeInit(DMA2_Stream1);
	// DMA_InitStructure.DMA_Channel = DMA_Channel_5;							// 通道选择
	// DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART6->DR;		/// DMA外设地址
	// DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)DMA_USART6_RX_BUF;		// DMA 存储器0地址
	// DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;					// 存储器到外设模式
	// DMA_InitStructure.DMA_BufferSize = DMA_USART6_RX_MAX_LEN;						// 数据传输量
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
	// DMA_Init(DMA2_Stream1, &DMA_InitStructure);
	// DMA_Cmd(DMA2_Stream1, ENABLE); // 使能DMA2_Stream5通道

	DMAx_Init(DMA2_Stream6,DMA_Channel_5,(uint32_t)&USART6->DR,(u32)DMA_USART6_TX_BUF,DMA_USART6_TX_BUF_LEN,DMA_DIR_MemoryToPeripheral);

	// DMA_DeInit(DMA2_Stream6); // 初始化DMA Stream
	// while (DMA_GetCmdStatus(DMA2_Stream6) != DISABLE)
	// 	; // 等待DMA可配置
	// /* 配置DMA1 Stream6，USART6发送 */
	// DMA_InitStructure.DMA_Channel = DMA_Channel_5;							// 通道选择
	// DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART6->DR;			// DMA外设地址
	// DMA_InitStructure.DMA_Memory0BaseAddr = (u32)DMA_USART6_TX_BUF;			// DMA 存储器0地
	// DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;					// 存储器到外设模式
	// DMA_InitStructure.DMA_BufferSize = DMA_USART6_TX_BUF_LEN;				// 数据传输量
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

	// DMA_Init(DMA2_Stream6, &DMA_InitStructure); // 初始化DMA Stream6

	DMA_ITConfig(DMA2_Stream6, DMA_IT_TC, ENABLE); // DMA1传输完成中断
	DMA_Cmd(DMA2_Stream6, DISABLE);				   // 不使能
	USART_Cmd(USART6, ENABLE);					   // 使能串口6
}

__attribute__((noinline)) void TOF_400F_init(void)
{
	u16 crc_16;
 //115200
  DMA_USART6_TX_BUF[0] = 0x01;
  DMA_USART6_TX_BUF[1] = 0x06;
  DMA_USART6_TX_BUF[2] = 0x00;
  DMA_USART6_TX_BUF[3] = 0x03;
  DMA_USART6_TX_BUF[4] = 0x00;
  DMA_USART6_TX_BUF[5] = 0x03;
  crc_16 = crc16_calculate_modbus(DMA_USART6_TX_BUF,6);
  DMA_USART6_TX_BUF[6] = (u8)crc_16;
  DMA_USART6_TX_BUF[7] = crc_16 >> 8;
  DMA_USARTx_Send(DMA2_Stream6,8);
  delay_ms(10);

 //50ms发送一次
//   DMA_USART6_TX_BUF[0] = 0x01;
//   DMA_USART6_TX_BUF[1] = 0x06;
//   DMA_USART6_TX_BUF[2] = 0x00;
  DMA_USART6_TX_BUF[3] = 0x05;
  DMA_USART6_TX_BUF[4] = 0x00;
  DMA_USART6_TX_BUF[5] = 0x32;
  crc_16 = crc16_calculate_modbus(DMA_USART6_TX_BUF,6);
  DMA_USART6_TX_BUF[6] = (u8)crc_16;
  DMA_USART6_TX_BUF[7] = crc_16 >> 8;
  DMA_USARTx_Send(DMA2_Stream6,8);
  delay_ms(10);

 //距离模式
//   DMA_USART6_TX_BUF[0] = 0x01;
//   DMA_USART6_TX_BUF[1] = 0x06;
//   DMA_USART6_TX_BUF[2] = 0x00;
  DMA_USART6_TX_BUF[3] = 0x04;
  DMA_USART6_TX_BUF[4] = 0x00;
  DMA_USART6_TX_BUF[5] = 0x00;//00高精度，01长距离
  crc_16 = crc16_calculate_modbus(DMA_USART6_TX_BUF,6);
  DMA_USART6_TX_BUF[6] = (u8)crc_16;
  DMA_USART6_TX_BUF[7] = crc_16 >> 8;
  DMA_USARTx_Send(DMA2_Stream6,8);
  delay_ms(10);

}



void USART6_IRQHandler(void) // 串口6中断服务程序
{
  static u8 error_flag = 0;
  static u16 crc16_modbus,rx_dis;
  if (USART_GetITStatus(USART6, USART_IT_IDLE) != RESET) // 空闲中断触发
  {

    com6_recv_end_flag = 1; // 接受完成标志位置1

    DMA_Cmd(DMA2_Stream1, DISABLE); /* 暂时关闭dma，数据尚未处理 */

    com6_rx_len = DMA_USART6_RX_MAX_LEN - DMA_GetCurrDataCounter(DMA2_Stream1); /* 获取接收到的数据长度 单位为字节*/

    DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_TCIF1); /* 清DMA标志位 */
                                                 // DMA_SetCurrDataCounter(DMA2_Stream1,DMA_USART6_RX_MAX_LEN);/* 重新赋值计数值，必须大于等于最大可能接收到的数据帧数目 */
									 
    if (com6_rx_len >= 7)
    {

		
		crc16_modbus = DMA_USART6_RX_BUF[com6_rx_len - 1];
		crc16_modbus <<= 8;
		crc16_modbus |= DMA_USART6_RX_BUF[com6_rx_len - 2];
		// print_debug("%x %x %x %x ",crc16_modbus, DMA_USART6_RX_BUF[com6_rx_len - 2] ,DMA_USART6_RX_BUF[com6_rx_len - 1]);
		if(crc16_modbus == crc16_calculate_modbus(DMA_USART6_RX_BUF,com6_rx_len - 2))
		{
            if (com6_rx_len == 7 && DMA_USART6_RX_BUF[1] == 0x03) {
                rx_dis = DMA_USART6_RX_BUF[3];
                rx_dis <<= 8;
                rx_dis |= DMA_USART6_RX_BUF[4];
                if (rx_dis > 5000) {
                    rx_dis               = 0;
                    distance_kalman.data = rx_dis / 1000.0;
                    kalman_filter(&distance_kalman);
                    distance_ = 0;
                } else {
                    distance_kalman.data = rx_dis / 1000.0;
                    kalman_filter(&distance_kalman);
                    distance_ = distance_kalman.x;
                }
				if(distance_ > 0.65)
				{
					distance_ = 0;
				}
                // print_debug("%d  %f \n",rx_dis,distance_);
            }
        }

		com6_rx_len = 0; // 清缓存



	  memset(DMA_USART6_RX_BUF, 0, LENARRAY(DMA_USART6_RX_BUF));
      DMA_SetCurrDataCounter(DMA2_Stream1, DMA_USART6_RX_MAX_LEN); // 重新设置传输的数据数量

      DMA_Cmd(DMA2_Stream1, ENABLE); // 开启DMA传输
    }



	else
    {

      if (error_flag)//接收错误
      {
        // Usart_SendArray(USART1,com1_rx_buffer,25);
        error_flag = 0;
		memset(DMA_USART6_RX_BUF, 0, LENARRAY(DMA_USART6_RX_BUF));
        DMA_SetCurrDataCounter(DMA2_Stream1, DMA_USART6_RX_MAX_LEN); // 重新设置传输的数据数量
        DMA_Cmd(DMA2_Stream1, ENABLE);                       // 开启DMA传输
      }
      else
      {
		static short int i;
        for (i = 1; i <= 10; i++)
        {
          if (DMA_USART6_RX_BUF[i] == 0xa5) // 寻找帧头位置，接收多余数据
          {
            memset(DMA_USART6_RX_BUF, 0, LENARRAY(DMA_USART6_RX_BUF));
            DMA_SetCurrDataCounter(DMA2_Stream1, i); // 重新设置传输的数据数量
            DMA_Cmd(DMA2_Stream1, ENABLE);                           // 开启DMA传输
            error_flag = 1;
            break;
          }
        }
		if(!error_flag)
		{
		memset(DMA_USART6_RX_BUF, 0, LENARRAY(DMA_USART6_RX_BUF));
      	DMA_SetCurrDataCounter(DMA2_Stream1, DMA_USART6_RX_MAX_LEN); // 重新设置传输的数据数量
      	DMA_Cmd(DMA2_Stream1, ENABLE); // 开启DMA传输
		}
      }
    }
	USART_ReceiveData(USART6); // 清除空闲中断标志位（接收函数有清标志位的作用）
  }



  if (USART_GetFlagStatus(USART6, USART_IT_TXE) == RESET) // 串口发送完成
  {
    USART_ITConfig(USART6, USART_IT_TC, DISABLE);
  }
}



void DMA2_Stream6_IRQHandler(void)
{
	//清除标志
	if(DMA_GetFlagStatus(DMA2_Stream6,DMA_FLAG_TCIF6)!=RESET)//等待DMA传输完成
	{
		DMA_ClearFlag(DMA2_Stream6,DMA_FLAG_TCIF6); //清除DMA传输完成标志
   		DMA_Cmd(DMA2_Stream6,DISABLE);				//关闭使能
    	USART_ITConfig(USART6,USART_IT_TC,ENABLE);  //打开串口发送完成中断
	}
}
#endif
