/*
*.............................
*write by xujunhui
*https://github.com/Kirinnana
*test zhouwentao
*hardware zhangjun
*/
#include "system.h"

__attribute__((noinline)) void NVIC_Configuration(void)
{

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//设置NVIC中断分组2:2位抢占优先级，2位响应优先级

}


// 自定义gcvt函数
char *my_gcvt(double value, int ndigit, char *buf)
{
	static int sign;			   // 符号标志
	static int integer, linteger; // 整数部分
	static double fraction ,lvalue;	   // 小数部分
	static int count, k;		   // 计数器
	static int i;				   // 循环变量

	// 判断符号
	if (value < 0)
	{
		sign = 1;		// 负数
		value = -value; // 取绝对值
		buf[0] = '-';	// 添加负号
	}
	else
	{
		sign = 0; // 正数或零
	}

	// 判断零
	if (value == 0)
	{
		buf[sign] = '0';	  // 添加零字符
		buf[sign + 1] = '\0'; // 添加结束符
		return buf;			  // 返回字符串指针
	}

	// 判断整数
	integer = (int)value;		// 取整数部分
	fraction = value - integer; // 取小数部分
	if (fraction == 0)
	{
		sprintf(buf + sign, "%d", integer); // 直接转换整数部分为字符串
		return buf;							// 返回字符串指针
	}
	// 处理小数
	if (integer == 0)
	{
		buf[sign] = '0';
		sign++;
		count = 0, k = -1; // 初始化计数器

		while (fraction != 0 && count < ndigit)
		{								// 循环乘以10直到小数部分为零或者达到最大位数
			value *= 10;				// 乘以10
			integer = (int)value;		// 取整数部分
			fraction = value - integer; // 取小数部分
			count++;					// 计数器加一
		}

		// 处理四舍五入
		if (fraction >= 0.5)
		{			   // 如果小数部分大于等于0.5，就进位
			integer++; // 整数部分加一
			if (integer == pow(10, count))
			{			 // 如果整数部分等于10的count次方，说明进位后多了一位
				count++; // 计数器加一
			}
		}
		linteger = integer;
		while (integer > 0)
		{
			integer /= 10; // 将num除以10

			k++; // 将len加一
		}

		for (i = k; i < count; i++)
		{
			buf[sign + i - 1] = '0';
			//		printf("%c  %d\n",buf[sign + k],sign + k);
		}

		// 转换为字符串
		sprintf(buf + sign + count - k -1, "%d", linteger); // 转换整数部分为字符串

		// 插入小数点

		for (i = sign + count; i > sign; i--)
		{ // 从后往前移动count位字符，空出一位插入小数点
			buf[i] = buf[i - 1];
		}
		buf[sign] = '.'; // 插入小数点

		// 删除末尾的小数点或空格（如果有的话）
		i = sign + count; // 定位到末尾字符的位置

		while (buf[i] == '.' || buf[i] == ' ' || buf[i] == '0')
		{ // 如果是小数点或空格，就删除它
			buf[i] = '\0';
			i--;
		}
		buf[i + 1] = '\0';
		//	printf("%d\n",k);
		return buf; // 返回字符串指针
	}

	count = 0, k = 0; // 初始化计数器
	while (fraction != 0 && count < ndigit)
	{								// 循环乘以10直到小数部分为零或者达到最大位数
		value *= 10;
		lvalue = value;		// 乘以10
		integer = (int)lvalue;		// 取整数部分
		fraction = value - integer; // 取小数部分
		count++;					// 计数器加一
	}

	// 处理四舍五入
	if (fraction >= 0.5)
	{			   // 如果小数部分大于等于0.5，就进位
		integer++; // 整数部分加一
		if (integer == pow(10, count))
		{			 // 如果整数部分等于10的count次方，说明进位后多了一位
			count++; // 计数器加一
		}
	}

	// 转换为字符串
	sprintf(buf + sign, "%d", integer); // 转换整数部分为字符串

	while (integer > 0)
	{
		integer /= 10; // 将num除以10
		k++;		   // 将len加一
	}

	// 插入小数点

	for (i = sign + k; i > (sign + k) - count; i--)
	{ // 从后往前移动count位字符，空出一位插入小数点
		buf[i] = buf[i - 1];
	}
	buf[i] = '.'; // 插入小数点

	// 删除末尾的小数点或空格（如果有的话）
	i = sign + k; // 定位到末尾字符的位置

	while (buf[i] == '.' || buf[i] == ' ' || buf[i] == '0')
	{ // 如果是小数点或空格，就删除它
		buf[i] = '\0';
		i--;
	}
	buf[i + 1] = '\0';
	
	return buf; // 返回字符串指针
}

__attribute__((noinline)) long int time_difference(long int  last_second ,int last_millisencond ,long int  second_count ,int millisencond_count ,int arr)
{
	if (last_second == second_count)
			return millisencond_count - last_millisencond ;
	if (last_second +1 == second_count)
		return millisencond_count + arr - last_millisencond ;
	
	if (last_second  < second_count)
		return (millisencond_count + arr * second_count) - (last_millisencond + arr * last_second) ;
	return 0 ;

}

//校验和检查
__inline uint8_t CHeck(uint8_t *data,int number)
{
	static uint8_t sum,i;
	sum = 0;
	for(i=0;i<number;i++)
	 sum+=*(data + i);
	return sum;
}

// 实现计算抛物线方程的函数
__attribute__((noinline)) int findParabola(Point p1, Point p2, Point p3, double *a, double *b, double *c) {
    if (p1.x == p2.x || p2.x == p3.x || p1.x == p3.x)
    {
        return 0;
    }    
    double matrix[3][3] = {
        {p1.x * p1.x, p1.x, 1},
        {p2.x * p2.x, p2.x, 1},
        {p3.x * p3.x, p3.x, 1}
    };
    double det = matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) -
                 matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]) +
                 matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);

    double inv[3][3] = {
        {(matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) / det, 
         -(matrix[0][1] * matrix[2][2] - matrix[0][2] * matrix[2][1]) / det, 
         (matrix[0][1] * matrix[1][2] - matrix[0][2] * matrix[1][1]) / det},
        {-(matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]) / det, 
         (matrix[0][0] * matrix[2][2] - matrix[0][2] * matrix[2][0]) / det, 
         -(matrix[0][0] * matrix[1][2] - matrix[0][2] * matrix[1][0]) / det},
        {(matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]) / det, 
         -(matrix[0][0] * matrix[2][1] - matrix[0][1] * matrix[2][0]) / det, 
         (matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]) / det}
    };

    *a = inv[0][0] * p1.y + inv[0][1] * p2.y + inv[0][2] * p3.y;
    *b = inv[1][0] * p1.y + inv[1][1] * p2.y + inv[1][2] * p3.y;
    *c = inv[2][0] * p1.y + inv[2][1] * p2.y + inv[2][2] * p3.y;
    return 1;
}

__attribute__((noinline)) void print_debug(const char *format, ...) 
{
	volatile static u16 i = 0;
	static char *buff;
	buff = (char *)DMA_USART1_TX_BUF;
    va_list ap;
    va_start(ap, format);
	while (!DMA_USART1_TX_flage) // 串口发送完成
	{
	}
    sprintf(buff,"[debug@%04ld:%03d]: ",second_count,millisencond_count);
	i = 18;
	while(buff[i] != '\0')
	{
		i++;
	}
    vsprintf(buff + i,format, ap);
	i += strlen(buff + i);
	// while(buff[i] != '\0')
	// {
	// 	i++;
	// }
	DMA_USART1_TX_flage = 0;
	DMA_USARTx_Send(DMA2_Stream7, i);			
    va_end(ap);
}

__attribute__((noinline)) int my_printf(const char *format, ...) 
{
	volatile static u16 i = 0;
	static char *buff;
	buff = (char *)DMA_USART1_TX_BUF;
    va_list ap;
    va_start(ap, format);
	while (!DMA_USART1_TX_flage) // 串口发送完成
	{
	}		
	i = 0;
    vsprintf(buff + i,format, ap);
	i += strlen(buff + i);
	// while(buff[i] != '\0')
	// {
	// 	i++;
	// }
	DMA_USART1_TX_flage = 0;
	DMA_USARTx_Send(DMA2_Stream7, i);		
    va_end(ap);
	return 0;
}

/***********************************************************************************************
*函数名: uint16 crc16_calculate_modbus(uint8 *ptr, uint16 len)
*功能描述: modbus crc16校验
*特别说明:
*函数参数:
*函数返回值:
*修改记录:
***********************************************************************************************/
static const uint8 a_crc16mTableHi[] = {
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
	0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
	0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
	0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
	0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
	0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40
};
static const uint8 a_crc16mTableLo[] = {
	0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
	0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
	0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
	0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
	0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
	0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
	0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
	0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
	0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
	0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
	0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
	0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
	0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
	0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
	0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
	0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
	0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
	0x40
};
__inline uint16 crc16_calculate_modbus(uint8 *p_data, uint16 len)
{
	static uint8 crchi = 0xff;
	static uint8 crclo = 0xff;
	static uint16 index;
	crchi = 0xff;
	crclo = 0xff;
	while (len--)
	{
		index = crclo ^ *p_data++;
		crclo = crchi ^ a_crc16mTableHi[index];
		crchi = a_crc16mTableLo[index];
	}
	return (crchi << 8 | crclo);
}

__inline double fast_sqrt(double ptr)
{
	return __sqrtf(ptr);
}

// C函数来处理硬件故障
__attribute__((noinline)) void hard_fault_handler_c(HardFaultStackFrame *psp,HardFaultStackFrame *msp,const char* str) 
{
    uint32_t cfsr = SCB->CFSR;
    uint32_t hfsr = SCB->HFSR;
    uint32_t mmar = SCB->MMFAR;

    printf("\r\n");

    printf("%s cfsr :0x%x hfsr :0x%x mmar :0x%x\n\r",str, cfsr, hfsr, mmar);

	printf("\r\n psp \n\r 0x%x", psp);

    printf("\r\n");
    printf("r0: 0x%x r1: 0x%x r2: 0x%x r3: 0x%x r12: 0x%x lr: 0x%x pc: 0x%x psr: 0x%x\r\n",psp->R0,psp->R1,psp->R2,psp->R3,psp->R12,psp->PC,psp->PC,psp->PSR);


	printf("\r\n msp \n\r 0x%x", msp);

    printf("\r\n");
    printf("r0: 0x%x r1: 0x%x r2: 0x%x r3: 0x%x r12: 0x%x lr: 0x%x pc: 0x%x psr: 0x%x\r\n",msp->R0,msp->R1,msp->R2,msp->R3,msp->R12,msp->PC,msp->PC,msp->PSR);

    // 处理硬件故障
    // while (1) {
    // }
}

__attribute__((noinline)) void car_stop(void)
{
    drive_flag       = 0;
    PS2_control_flag = 0;
    target_speed     = 0;
    Angle_speed      = 0;
    front_back_wheel = 0;
    car_speed        = 0;
    car_Angle        = 0;
    pstwo_speed      = 0;
    pstwo_Angle      = 0;
}
