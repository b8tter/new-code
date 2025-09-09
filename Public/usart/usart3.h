/*
*.............................
*write by xujunhui
*https://github.com/Kirinnana
*test zhouwentao
*hardware zhangjun
*/
#ifndef __USART3_H
#define __USART3_H

#include "system.h" 

#ifdef CAMERA 
void USART3_Config(void);
void USART3_IRQHandler(void);
void Change_mode(u8 mode,short int dealy_time);//切换摄像头检测模式
#endif

#endif


