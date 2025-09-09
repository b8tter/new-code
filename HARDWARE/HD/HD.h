#ifndef __HD_H
#define __HD_H
#include "system.h"

#ifdef HUI_DU
void HDInit(void);
#define HD_2_11 PFin(0)
#define HD_2_10 PFin(1)
#define HD_2_9 PFin(2)
#define HD_2_8 PFin(3)
#define HD_2_7 PFin(4)
#define HD_2_6 PFin(5)
#define HD_2_5 PFin(6)
#define HD_2_4 PFin(7)
#define HD_2_3 PFin(8)
#define HD_2_2 PFin(9)
#define HD_2_1 PFin(10)
#define HD_2_0 PFin(11)
#define HD_2 {HD_2_0, HD_2_1, HD_2_2, HD_2_3, HD_2_4,HD_2_5,HD_2_6,HD_2_7,HD_2_8,HD_2_9,HD_2_10,HD_2_11}
#endif

void motor(int L_Lun, int R_Lun); // 设置左右轮速度
void Timer_Init(void);       // 定时器初始化
