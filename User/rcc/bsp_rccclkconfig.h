/*
*.............................
*write by xujunhui
*https://github.com/Kirinnana
*test zhouwentao
*hardware zhangjun
*/
#ifndef __BSP_RCCCLKCONFIG_H
#define __BSP_RCCCLKCONFIG_H

#include "stm32f4xx.h"

void RCC_HSE_Config(u32 pllm,u32 plln,u32 pllp,u32 pllq);

void MCO_GPIO_Config(void);

#endif /*__BSP_RCCCLKCONFIG_H */
