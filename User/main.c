/*
*.............................
*write by xujunhui
*https://github.com/Kirinnana
*test zhouwentao
*hardware zhangjun
*/


#include "system.h"

/* 告知连接器不从C库链接使用半主机的函数 */
//#pragma import(__use_no_semihosting)

/* 定义 _sys_exit() 以避免使用半主机模式 */

#define AC6_ENABLE
// 	  
#ifdef AC6_ENABLE
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	
__ASM (".global __use_no_semihosting");      
#else                                                       //AC5_ENABLE
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)	
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 
#endif

void _sys_exit(int x)
{
	x = x;
}

/* 标准库需要的支持类型 */
/*
struct __FILE
{
	int handle;
};
*/
//FILE __stdout;

// 提供_ttywrch的实现，这个函数是用来处理字符的
void _ttywrch(int ch) {
    ch = ch;
}

/*******************************************************************************
 * 函 数 名         : main
 * 函数功能		   : 主函数
 * 输    入         : 无
 * 输    出         : 无
 *******************************************************************************/
long int last_second = 0;
int last_millisencond = 0;
volatile u16 LED_count = 0;
volatile u16 psc_flag = 0, psc_flag_1 = 0;
// #define MONITORING_COLOCK //查看空闲时钟


int main()
{
	RCC_HSE_Config(8, 452, 2, 7); // 8,432,2, 超频 主频为8（外部晶振）/ 8 * 452 /2 默认336
	SysTick_Init(226);
	NVIC_Configuration();
	// 串口初始化
	USART_Config();

	LED_Init();
	LED_FLASH_Init();
	my_mem_init(SRAMIN); // 初始化内部内存池
	my_mem_init(SRAMEX);
	my_mem_init(SRAMCCM); // 初始化CCM内存池

	
#ifdef OLED	
	// oled初始化
	OLED_Init();
	OLED_ColorTurn(0);	 // 0正常显示，1 反色显示
	OLED_DisplayTurn(1); // 0正常显示 1 屏幕翻转显示
	volatile u16 oled_Refresh_count = 0;
#endif
	// 对于stm32f407 168M主频，因为系统初始化SystemInit函数里初始化APB1总线时钟为4分频即42M，所以TIM2~TIM7、TIM12~TIM14的时钟为APB1的时钟的两倍即84M；
	// APB2总线时钟为2分频即84M，TIM1、TIM8~TIM11的时钟为APB2时钟的两倍即168M
	
#ifdef CAMERA
	USART3_Config();
#endif
#ifdef TOF400F
	USART6_Config();//激光
	TOF_400F_init();
#endif

	delay_ms(10);

	TIM1_PWM_Init(11300 - 1, 20 - 1, 0x40); // 用于频率10Khz电机调速
	DIR_Init();//电机方向控制

	TIM5_Encoder_Init(); // 四个编码器中断初始化
	TIM4_Encoder_Init();
	TIM3_Encoder_Init();
	TIM2_Encoder_Init();

#ifdef PS_TWO
	PS2_Init();	   // ps2 手柄初始化
	PS2_SetInit(); // 锁定模式
	delay_ms(10);
	PS2_key_scan(); // ps2 按键扫描处理
#endif

	delay_ms(10);
	My_EXTI_Init();//按钮

#ifdef ULTRASONIC
	ultrasonic_Init(); // 超声波
#endif
	
	uart2_init(115200); // 用于在uart2hand出mpu数据
	delay_ms(10);
	MPU_Init(); // mpu自检

	delay_ms(50);

	RobotStatus_init();
	target_speed = 0;
	Angle_speed = 0;
	// front_back_wheel = 0;
	car_speed = 0;
	car_Angle = 0;
	pstwo_speed = 0;
	pstwo_Angle = 0;
	PS2_control_flag = 0;
#ifdef ULTRASONIC
	TIM6_Int_Init(50 - 1, 226 - 1);  // TIM6 0.1ms;
#else
	TIM6_Int_Init(500 - 1, 226 - 1);  // TIM6 1ms;
#endif


	TIM7_Int_Init(2500 - 1, 226 - 1); // pid频率200hz  pid计算pwm输出中断;

	delay_ms(100);
	
	// IWDG_Init(4, 800); // 只要在1280ms内进行喂狗就不会复位系统

	key1_flag = 0;
	key0_flag = 0;

#ifdef HUI_DU
	HDInit(); // 灰度传感器
#endif

#ifdef OLED	
	OLED_Clear();
#endif	

	// IWDG_FeedDog(); // 喂狗
#if GY25Z
	// 陀螺仪校准
	MPU_GY25Z_adjust();
#endif
#if JY901S
	MPU_JY_901S_zero();
#endif

	delay_ms(50);

#ifdef MONITORING_COLOCK
	volatile u32 free_clock;
#endif

	print_debug("start_run_main !!! \r\n");
	LED_FLASH_state_set(1);

	while (1)
	{

		if (last_millisencond != millisencond_count && (millisencond_count % 10) == 1) // 通过定时中断实现的精准延时
		{

			last_millisencond = millisencond_count;
			// last_second = second_count;
			state_machine_main(); // 状态机，机器人行为
			psc_flag_1 = !psc_flag_1;
#ifdef MONITORING_COLOCK
			printf("%d\n",free_clock);
			free_clock = 0;
#endif
            if (psc_flag_1) {

#ifdef OLED
                if (LED_count % 8 < 2) {
                    oled_show(LED_count % 8);
                    oled_Refresh_count = 0;

                } else
#endif
                {
#ifdef PS_TWO
                    if (!drive_flag && LED_count % 3 == 1) {

                        PS2_key_scan(); // ps2 按键扫描处理

                    }
#if !PID_DEBUG_FLAGE
                    else if (LED_count % 9 == 1)
#else
                    else if (LED_count % 4 == 1)
#endif
                    {

                        PS2_key_scan(); // ps2 按键扫描处理
                    }
#endif

#ifdef OLED
                    if (LED_count % 8 < 6) {
                        oled_show_Refresh(oled_Refresh_count, oled_Refresh_count + 1); // 更新2x8 = 16 行
                        oled_Refresh_count += 2;
                    }
#endif

                }

                LED_count++;
				if (LED_count >= 32)
				{
					// if(Temperature_out != 0 && reset_flag == 0)
					if (reset_flag)
					{

						NVIC_SystemReset();
						
					}
					LED1 = !LED1; // D1状态取反
					LED_count = 0;
					// printf("%d  \n",my_mem_perused(0));//查看内春使用率；
				}
            }
        }
#ifdef MONITORING_COLOCK
		free_clock++;
#endif				
	}
}
