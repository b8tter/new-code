#include "PWM.h"
#include "DIR.h"

__attribute__((noinline)) void TIM1_PWM_Init(u16 per, u16 psc, u16 deadtime)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	//TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);  // TIM1时钟使能
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOG , ENABLE); // 使能PORTA时钟

	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);	 // GPIOE8复用为定时器1
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1); // GPIOE9复用为定时器1
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1); // GPIOE10复用为定时器1
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1); // GPIOE11复用为定时器1

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;										// 复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;										// GPIO_OType_PP;     //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;									    //上拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;									// 速度50MHz
	GPIO_Init(GPIOE, &GPIO_InitStructure);												

	TIM_TimeBaseStructure.TIM_Prescaler = psc;					// 定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数模式
	TIM_TimeBaseStructure.TIM_Period = per;						// 自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); // 初始化定时器1

	// 初始化TIM1 Channel1,2,3,4 PWM模式
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;			  // 选择定时器模式:TIM脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 比较输出使能
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OutputNState_Disable; // 关闭互补输出
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High ; // 输出极性高

	TIM_OC1Init(TIM1, &TIM_OCInitStructure); // 根据T指定的参数初始化外设OC1
	TIM_OC2Init(TIM1, &TIM_OCInitStructure); // 根据T指定的参数初始化外设OC2
	TIM_OC3Init(TIM1, &TIM_OCInitStructure); // 根据T指定的参数初始化外设OC3
	TIM_OC4Init(TIM1, &TIM_OCInitStructure); // 根据T指定的参数初始化外设OC4

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable); // 使能TIM1在CCR1上的预装载寄存器
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable); // 使能TIM1在CCR2上的预装载寄存器
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable); // 使能TIM1在CCR3上的预装载寄存器
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable); // 使能TIM1在CCR4上的预装载寄存器

	TIM_BDTRInitTypeDef TIM_BDTRInitStruct;
	TIM_BDTRInitStruct.TIM_OSSRState = TIM_OSSRState_Enable ;
	TIM_BDTRInitStruct.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRInitStruct.TIM_LOCKLevel = TIM_LOCKLevel_OFF ;
	TIM_BDTRInitStruct.TIM_DeadTime = deadtime; // 死区时间，单位为定时器时钟周期
	TIM_BDTRInitStruct.TIM_Break = TIM_Break_Enable;
	TIM_BDTRInitStruct.TIM_BreakPolarity = TIM_BreakPolarity_High;
	TIM_BDTRInitStruct.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStruct);

	TIM_ARRPreloadConfig(TIM1, ENABLE); // ARPE使能
	TIM_CtrlPWMOutputs(TIM1, ENABLE);	// 使能TIM1的PWM输出
	TIM_Cmd(TIM1, ENABLE);				// 使能TIM1
}

//电机1PWM速度控制
void MOTOR_1_SetSpeed(int16_t speed) // speed:-7200~7200
{
    uint16_t temp;

    if(speed > 0)//正转
	{
		GPIO_ResetBits(GPIOD, GPIO_Pin_15);
	    GPIO_SetBits(GPIOG, GPIO_Pin_3);
		temp = speed;	
	}
	else if(speed < 0)//反转
	{
		GPIO_ResetBits(GPIOD, GPIO_Pin_15);
	    GPIO_SetBits(GPIOG, GPIO_Pin_3);
		temp = (-speed);
	}
	else//停止
	{
		GPIO_ResetBits(GPIOD, GPIO_Pin_15);
	    GPIO_ResetBits(GPIOG, GPIO_Pin_3);
		temp = 0;
	}
	
	TIM_SetCompare1(TIM1,temp);
}
//电机2PWM速度控制
void MOTOR_2_SetSpeed(int16_t speed) // speed:-7200~7200
{
    uint16_t temp;

    if(speed > 0)//正转
    {
        GPIO_ResetBits(GPIOD, GPIO_Pin_14);
        GPIO_SetBits(GPIOG, GPIO_Pin_2);
        temp = speed;	
    }
    else if(speed < 0)//反转
    {
        GPIO_ResetBits(GPIOD, GPIO_Pin_14);
        GPIO_SetBits(GPIOG, GPIO_Pin_2);
        temp = (-speed);
    }
    else//停止
    {
        GPIO_ResetBits(GPIOD, GPIO_Pin_14);
        GPIO_ResetBits(GPIOG, GPIO_Pin_2);
        temp = 0;
    }
    
    TIM_SetCompare2(TIM1,temp);
}
//电机3PWM速度控制
void MOTOR_3_SetSpeed(int16_t speed) // speed:-7200~7200
{
    uint16_t temp;

    if(speed > 0)//正转
    {
        GPIO_ResetBits(GPIOE, GPIO_Pin_0);
        GPIO_SetBits(GPIOE, GPIO_Pin_2);
        temp = speed;	
    }
    else if(speed < 0)//反转
    {
        GPIO_ResetBits(GPIOE, GPIO_Pin_0);
        GPIO_SetBits(GPIOE, GPIO_Pin_2);
        temp = (-speed);
    }
    else//停止
    {
        GPIO_ResetBits(GPIOE, GPIO_Pin_0);
        GPIO_ResetBits(GPIOE, GPIO_Pin_2);
        temp = 0;
    }
    
    TIM_SetCompare3(TIM1,temp);
}
//电机4PWM速度控制
void MOTOR_4_SetSpeed(int16_t speed) // speed:-7200~7200
{
	uint16_t temp;

	if(speed > 0)//正转
	{
		GPIO_ResetBits(GPIOE, GPIO_Pin_1);
		GPIO_SetBits(GPIOE, GPIO_Pin_3);
		temp = speed;	
	}
	else if(speed < 0)//反转
	{
		GPIO_ResetBits(GPIOE, GPIO_Pin_1);
		GPIO_SetBits(GPIOE, GPIO_Pin_3);
		temp = (-speed);
	}
	else//停止
	{
		GPIO_ResetBits(GPIOE, GPIO_Pin_1);
		GPIO_ResetBits(GPIOE, GPIO_Pin_3);
		temp = 0;
	}
	
	TIM_SetCompare4(TIM1,temp);
}