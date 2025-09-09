/*
*.............................
*write by xujunhui
*https://github.com/Kirinnana
*test zhouwentao
*hardware zhangjun
*/
#include "bsp_usart.h"
#include "system.h"
float rx_num;
#define USART_MAX_LEN 12
#define DMA_USART1_TX_BUF_LEN 256
volatile uint16_t DMA_USART1_RX_len = 0;	   // 接收帧数据的长度
volatile uint8_t DMA_USART1_recv_end_flag = 0; // 帧数据接收完成标志
uint8_t DMA_USART1_RX_BUF[12] = {0};		   // 接收数据缓存
uint8_t DMA_USART1_TX_BUF[256] = {0};		   // DMA发送缓存
volatile u8 DMA_USART1_TX_flage = 1;

__attribute__((noinline)) void USART_Config(void)
{
	// GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  // 使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // 使能USART1时钟

	// 串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);  // GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1); // GPIOA10复用为USART1

	// USART1端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; // GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			// 复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		// 速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			// 推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			// 上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);					// 初始化PA9，PA10

	// USART1 初始化设置
	USART_InitStructure.USART_BaudRate = DEBUG_USART_BAUDRATE;						// 波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						// 字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							// 一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;								// 无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// 收发模式
	USART_Init(USART1, &USART_InitStructure);										// 初始化串口1

	// USART_Cmd(USART1, ENABLE);  //使能串口1

	// USART_ClearFlag(USART1, USART_FLAG_TC);

	// USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断

	// //Usart1 NVIC 配置
	// NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	// NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级3
	// NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//子优先级3
	// NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	// NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		  // 串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; // 抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		  // 子优先级1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);							  // 根据指定的参数初始化VIC寄存器、

	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream5_IRQn;	  // 嵌套通道为DMA2_Stream5_IRQn
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; // 抢占优先级为 2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  // 响应优先级为 2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // 通道中断使能
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;	  // 串口2发送中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; // 抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		  // 子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);

	// DMA_InitTypeDef DMA_InitStructure;

	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE); // 开启串口空闲中断
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE); // 开启串口DMA接收
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE); // 开启串口DMA接收
												   /* 配置串口DMA接收*/

	DMAx_Init(DMA2_Stream5,DMA_Channel_4,(uint32_t)&USART1->DR,(uint32_t)DMA_USART1_RX_BUF,USART_MAX_LEN,DMA_DIR_PeripheralToMemory);

	// RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE); // 开启DMA时钟
	// DMA_DeInit(DMA2_Stream5);
	// DMA_InitStructure.DMA_Channel = DMA_Channel_4;							// 通道选择
	// DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;		/// DMA外设地址
	// DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)DMA_USART1_RX_BUF;	// DMA 存储器0地址
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
	// DMA_Init(DMA2_Stream5, &DMA_InitStructure);
	// DMA_Cmd(DMA2_Stream5, ENABLE); // 使能DMA2_Stream5通道

	DMAx_Init(DMA2_Stream7,DMA_Channel_4,(uint32_t)&USART1->DR,(u32)DMA_USART1_TX_BUF,DMA_USART1_TX_BUF_LEN,DMA_DIR_MemoryToPeripheral);

	// DMA_DeInit(DMA2_Stream7); // 初始化DMA Stream
	// while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE)
	// 	; // 等待DMA可配置
	// /* 配置DMA2 Stream6，USART1发送 */
	// DMA_InitStructure.DMA_Channel = DMA_Channel_4;							// 通道选择
	// DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;			// DMA外设地址
	// DMA_InitStructure.DMA_Memory0BaseAddr = (u32)DMA_USART1_TX_BUF;			// DMA 存储器0地
	// DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;					// 存储器到外设模式
	// DMA_InitStructure.DMA_BufferSize = DMA_USART1_TX_BUF_LEN;				// 数据传输量
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

	// DMA_Init(DMA2_Stream7, &DMA_InitStructure); // 初始化DMA Stream6

	DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE); // DMA2传输完成中断
	DMA_Cmd(DMA2_Stream7, DISABLE);				   // 不使能
	USART_Cmd(USART1, ENABLE);					   // 使能串口1
}

/* 发送一个字节 */
__attribute__((noinline)) void Usart_SendByte(USART_TypeDef *pUSARTx, uint8_t data)
{
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET)
		;
	USART_SendData(pUSARTx, data);
}

/* 发送两个字节的数据 */
__attribute__((noinline)) void Usart_SendHalfWord(USART_TypeDef *pUSARTx, uint16_t data)
{
	uint8_t temp_h, temp_l;

	temp_h = (data & 0xff00) >> 8;
	temp_l = data & 0xff;
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET)
		;
	USART_SendData(pUSARTx, temp_h);
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET)
		;
	USART_SendData(pUSARTx, temp_l);
}

/* 发送8位数据的数组 */
__attribute__((noinline)) void Usart_SendArray(USART_TypeDef *pUSARTx, uint8_t *array, uint8_t num)
{
	uint8_t i;
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TC) == RESET)
		;
	for (i = 0; i < num; i++)
	{
		Usart_SendByte(pUSARTx, array[i]);
	}
}

/* 发送字符串 */
void Usart_SendStr(USART_TypeDef *pUSARTx, uint8_t *str)
{
	uint8_t i = 0;
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TC) == RESET)
		;
	do
	{
		Usart_SendByte(pUSARTx, *(str + i));
		i++;
	} while (*(str + i) != '\0');
}

/// 重定向c库函数printf到串口，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
	/* 等待发送完毕 */
	while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_TXE) == RESET)
		;
	/* 发送一个字节数据到串口 */
	USART_SendData(DEBUG_USARTx, (uint8_t)ch);

	return (ch);
}

/// 重定向c库函数scanf到串口，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{
	/* 等待串口输入数据 */
	while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_RXNE) == RESET)
		;

	return (int)USART_ReceiveData(DEBUG_USARTx);
}





void DEBUG_USART_IRQHandler(void) // 串口中断
{

	// static u8 error_flag = 0;
	if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET) // 空闲中断触发
	{

		DMA_USART1_recv_end_flag = 1; // 接受完成标志位置1

		DMA_Cmd(DMA2_Stream5, DISABLE); /* 暂时关闭dma，数据尚未处理 */

		DMA_USART1_RX_len = USART_MAX_LEN - DMA_GetCurrDataCounter(DMA2_Stream5); /* 获取接收到的数据长度 单位为字节*/

		DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_TCIF5); /* 清DMA标志位 */
													 // DMA_SetCurrDataCounter(DMA2_Stream5,USART_MAX_LEN);/* 重新赋值计数值，必须大于等于最大可能接收到的数据帧数目 */

		// Usart_SendArray(USART1,DMA_USART1_RX_BUF,11);

		if (DMA_USART1_RX_BUF[0] == 0xfe && DMA_USART1_RX_BUF[1] == 0xff)
		{
			memcpy(&rx_num, &DMA_USART1_RX_BUF[4], 4);
#ifdef OLED
			// OLED_ShowNum(8, 24, 1, 1, 12, 1);
#endif
			// target_speed = (int)rx_num;
			// motoer_1_Kp = rx_num;
            switch (DMA_USART1_RX_BUF[2]) {
                case 0x62:
                    car_stop();
                    break;
#if PID_DEBUG_FLAGE
                case 0x63:
                    target_speed = (int)rx_num;
                    break;
                case 0x64:
                    car_speed               = car_speed + 10;
                    break;
                case 0x65:
                    car_speed               = car_speed - 10;
                    break;
                case 0x66:
                    PID_DEBUG.Kp            = PID_DEBUG.Kp + 0.1;
                    PID_DEBUG.integralError = 0;
                    PID_DEBUG.lastError     = 0;

                    break;
                case 0x67:
                    PID_DEBUG.Kp            = PID_DEBUG.Kp - 0.1;
                    PID_DEBUG.integralError = 0;
                    PID_DEBUG.lastError     = 0;

                    break;
                case 0x68:
                    PID_DEBUG.Ki            = PID_DEBUG.Ki + 0.01;
                    PID_DEBUG.integralError = 0;
                    PID_DEBUG.lastError     = 0;

                    break;
                case 0x69:
                    PID_DEBUG.Ki            = PID_DEBUG.Ki - 0.01;
                    PID_DEBUG.integralError = 0;
                    PID_DEBUG.lastError     = 0;

                    break;
                case 0x70:
                    PID_DEBUG.Kd            = PID_DEBUG.Kd + 0.001;
                    PID_DEBUG.integralError = 0;
                    PID_DEBUG.lastError     = 0;

                    break;
                case 0x71:
                    PID_DEBUG.Kd = PID_DEBUG.Kd - 0.001;
					        PID_DEBUG.integralError = 0;
        PID_DEBUG.lastError = 0;
                    break;
#endif
                case 0x72:
                    encoder_Num[3] = encoder_Num[1] = encoder_Num[2] = encoder_Num[0] = 0.1;
                    break;
                case 0x73:
                    up_data_location(1.0, 2.0, 0, 3);

                    break;
                case 0x74:
                    Change_mode((u8)DMA_USART1_RX_BUF[3], 50);
                    break;
            }
            // memcpy(DMA_USART1_TX_BUF,DMA_USART1_RX_BUF,4);

			// DMA_USARTx_Send(DMA2_Stream7,4);
			// memset(DMA_USART1_TX_BUF, 0, LENARRAY(DMA_USART1_TX_BUF) + 1);
		}
		memset(DMA_USART1_RX_BUF, 0, LENARRAY(DMA_USART1_RX_BUF));
		DMA_SetCurrDataCounter(DMA2_Stream5, USART_MAX_LEN); // 重新设置传输的数据数量

		DMA_Cmd(DMA2_Stream5, ENABLE); // 开启DMA传输

		USART_ReceiveData(USART1); // 清除空闲中断标志位（接收函数有清标志位的作用）
	}

	if (USART_GetFlagStatus(USART1, USART_IT_TXE) == RESET) // 串口发送完成
	{
		USART_ITConfig(USART1, USART_IT_TC, DISABLE);
		DMA_USART1_TX_flage = 1;
	}
}

void DMA2_Stream7_IRQHandler(void)
{
	// 清除标志
	if (DMA_GetFlagStatus(DMA2_Stream7, DMA_FLAG_TCIF7) != RESET) // 等待DMA2_Steam7传输完成
	{
		DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7); // 清除DMA2_Steam7传输完成标志
		DMA_Cmd(DMA2_Stream7, DISABLE);				 // 关闭使能
		USART_ITConfig(USART1, USART_IT_TC, ENABLE); // 打开串口发送完成中断
	}
}
