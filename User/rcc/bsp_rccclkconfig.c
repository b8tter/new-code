/*
*.............................
*write by xujunhui
*https://github.com/Kirinnana
*test zhouwentao
*hardware zhangjun
*/
#include "bsp_rccclkconfig.h"

/*******************************************************************************
* 函 数 名         : RCC_HSE_Config
* 函数功能		   : 自定义系统时钟
* 输    入         : pllm：VCO 输入时钟 分频因子，范围0-63
					 plln：VCO 输出时钟 倍频因子，范围192-432
					 pllp：PLLCLK 时钟分频因子，范围2, 4, 6, or 8
					 pllq：OTG FS,SDIO,RNG 时钟分频因子，范围4-15
* 输    出         : 无
*******************************************************************************/
__attribute__((noinline)) void RCC_HSE_Config(u32 pllm,u32 plln,u32 pllp,u32 pllq)
{
		__IO uint32_t  HSEStartUpStatus = 0;//使用__IO，即VOlatie，不被编译器优化

	
	RCC_DeInit(); //将外设RCC寄存器重设为缺省值
	RCC_HSEConfig(RCC_HSE_ON);//设置外部高速晶振（HSE）
	if(RCC_WaitForHSEStartUp()==SUCCESS) //等待HSE起振
	{
		RCC_HCLKConfig(RCC_SYSCLK_Div1);//设置AHB时钟（HCLK）
		RCC_PCLK2Config(RCC_HCLK_Div2);//设置低速APB2时钟（PCLK2）
		RCC_PCLK1Config(RCC_HCLK_Div4);//设置低速APB1时钟（PCLK1）
		RCC_PLLConfig(RCC_PLLSource_HSE,pllm,plln,pllp,pllq);//设置PLL时钟源及倍频系数
		RCC_PLLCmd(ENABLE); //使能或者失能PLL
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY)==RESET);//检查指定的RCC标志位设置与否,PLL就绪
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);//设置系统时钟（SYSCLK）
		while(RCC_GetSYSCLKSource()!=0x08);//返回用作系统时钟的时钟源,0x08：PLL作为系统时钟
	}	
}

void MCO_GPIO_Config()
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;//推挽方式输出
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA, &GPIO_InitStruct);	
}
























