#ifndef __TIM_H
#define __TIM_H
#include "system.h"
void TIM1_PWM_Init(u16 per,u16 psc,u16 deadtime);
//per是自动重装载值决定PWM周期，psc代表预分频系数用于对定时器时钟进行分频，deadtime死区时间用于在互补输出时防止上下桥臂同时导通
void MOTOR_1_SetSpeed(int16_t speed);   
void MOTOR_2_SetSpeed(int16_t speed);
void MOTOR_3_SetSpeed(int16_t speed);   
void MOTOR_4_SetSpeed(int16_t speed);   

#endif