/*
*.............................
*write by xujunhui
*https://github.com/Kirinnana
*test zhouwentao
*hardware zhangjun
*/
#ifndef __USART6_H
#define __USART6_H

#include "system.h" 


#ifdef TOF400F
void USART6_Config(void);
void TOF_400F_init(void);
void USART6_IRQHandler(void);

#endif

#endif


