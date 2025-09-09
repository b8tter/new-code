#ifndef __ENCODER_H
#define __ENCODER_H

#ifdef __cplusplus
 extern "C" {
#endif


#include "system.h"

#define ENCODER_TIM_PSC 2-1        /*计数器分频*/
#define ENCODER_TIM_PERIOD 65535 /*计数器最大值*/
#define CNT_INIT 0               /*计数器初值*/
#define ENCODER_RESOLUTION 13//500    /*编码器一圈的物理脉冲数*/
#define ENCODER_MULTIPLE (4/(encoder_tim_psc + 1))       /*编码器倍频，通过定时器的编码器模式设置*/
#define MOTOR_REDUCTION_RATIO 44//30 /*电机的减速比*/

/*电机转一圈总的脉冲数(定时器能读到的脉冲数) = 编码器物理脉冲数*编码器倍频*电机减速比 */
#define TOTAL_RESOLUTION (ENCODER_RESOLUTION * ENCODER_MULTIPLE * MOTOR_REDUCTION_RATIO)

#define angularVelocity(a) 36000.0 / (float)TOTAL_RESOLUTION *a

// #define readSpeed(a) a * 0.745f / TOTAL_RESOLUTION * 15000
#define readSpeed(a) a / TOTAL_RESOLUTION * 12000.0 // 一分钟采12000次编码器的值

extern u8 encoder_tim_psc;

//奇数编号为右侧电机 偶数编号为左侧电机
extern double encoder_Num[4]; //电机1 编码器转转速值 encoder_Num[0] = encoder_Num1


extern double encoder_Num1_raw; //电机1 编码器原始值
extern double encoder_Num2_raw;//M2
extern double encoder_Num3_raw;//M3
extern double encoder_Num4_raw;//M4


void TIM5_Encoder_Init(void);
void TIM4_Encoder_Init(void);
void TIM3_Encoder_Init(void);
void TIM2_Encoder_Init(void);
void TIMx_Encoder_psc_reinit(TIM_TypeDef* TIMx);
void calc_motor_rotate_speed(void);

#endif
