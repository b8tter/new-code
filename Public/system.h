/*
*.............................
*write by xujunhui
*https://github.com/Kirinnana
*test zhouwentao
*hardware zhangjun
*/
#ifndef _system_H
#define _system_H
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
//滑动均值滤波
#include "meanfilter.h"

#include "stm32f4xx.h"
#include "led.h"
#include "bsp_rccclkconfig.h"
#include "SysTick.h"
// 串口1
#include "bsp_usart.h"
// 外部中断
#include "exti.h"
#include "dma.h"

#include "oled.h"
#include "key.h" //按键
#include "PWM.h"
#include "DIR.h"
#include "encoder.h" //编码器
#include "RobotStatus.h"
#include "control.h"    //控制
#include "Mpid.h"       //pid算法
#include "pid_debug.h"  //pid调试串口发送
#include "MPU.h"        //陀螺仪
#include "pstwo.h"      //ps2 手柄
#include "scan.h"       //扫描函数
#include "show.h"       //oled显示
#ifdef ULTRASONIC
#include "ultrasonic.h" //超声波传感器
#endif
#include "HD.h"
#include "odom.h"
#include "matrix_mul.h"
#include "malloc.h"
// #include "iwdg.h"//看门狗
#include "HW.h"//红外
#include "state_machines.h"
#include "usart3.h"	 //串口3
#include "usart6.h"




// 具体实现思想,参考<<CM3权威指南>>第五章(87页~92页).M4同M3类似,只是寄存器地址变了.
// IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000) + 0x2000000 + ((addr & 0xFFFFF) << 5) + (bitnum << 2))
#define MEM_ADDR(addr) *((volatile unsigned long *)(addr))
#define BIT_ADDR(addr, bitnum) MEM_ADDR(BITBAND(addr, bitnum))
// IO口地址映射
#define GPIOA_ODR_Addr (GPIOA_BASE + 20) // 0x40020014
#define GPIOB_ODR_Addr (GPIOB_BASE + 20) // 0x40020414
#define GPIOC_ODR_Addr (GPIOC_BASE + 20) // 0x40020814
#define GPIOD_ODR_Addr (GPIOD_BASE + 20) // 0x40020C14
#define GPIOE_ODR_Addr (GPIOE_BASE + 20) // 0x40021014
#define GPIOF_ODR_Addr (GPIOF_BASE + 20) // 0x40021414
#define GPIOG_ODR_Addr (GPIOG_BASE + 20) // 0x40021814
#define GPIOH_ODR_Addr (GPIOH_BASE + 20) // 0x40021C14
#define GPIOI_ODR_Addr (GPIOI_BASE + 20) // 0x40022014

#define GPIOA_IDR_Addr (GPIOA_BASE + 16) // 0x40020010
#define GPIOB_IDR_Addr (GPIOB_BASE + 16) // 0x40020410
#define GPIOC_IDR_Addr (GPIOC_BASE + 16) // 0x40020810
#define GPIOD_IDR_Addr (GPIOD_BASE + 16) // 0x40020C10
#define GPIOE_IDR_Addr (GPIOE_BASE + 16) // 0x40021010
#define GPIOF_IDR_Addr (GPIOF_BASE + 16) // 0x40021410
#define GPIOG_IDR_Addr (GPIOG_BASE + 16) // 0x40021810
#define GPIOH_IDR_Addr (GPIOH_BASE + 16) // 0x40021C10
#define GPIOI_IDR_Addr (GPIOI_BASE + 16) // 0x40022010

// IO口操作,只对单一的IO口
// 确保n的值小于16
#define PAout(n) BIT_ADDR(GPIOA_ODR_Addr, n) // 输出
#define PAin(n) BIT_ADDR(GPIOA_IDR_Addr, n)  // 输入

#define PBout(n) BIT_ADDR(GPIOB_ODR_Addr, n) // 输出
#define PBin(n) BIT_ADDR(GPIOB_IDR_Addr, n)  // 输入

#define PCout(n) BIT_ADDR(GPIOC_ODR_Addr, n) // 输出
#define PCin(n) BIT_ADDR(GPIOC_IDR_Addr, n)  // 输入

#define PDout(n) BIT_ADDR(GPIOD_ODR_Addr, n) // 输出
#define PDin(n) BIT_ADDR(GPIOD_IDR_Addr, n)  // 输入

#define PEout(n) BIT_ADDR(GPIOE_ODR_Addr, n) // 输出
#define PEin(n) BIT_ADDR(GPIOE_IDR_Addr, n)  // 输入

#define PFout(n) BIT_ADDR(GPIOF_ODR_Addr, n) // 输出
#define PFin(n) BIT_ADDR(GPIOF_IDR_Addr, n)  // 输入

#define PGout(n) BIT_ADDR(GPIOG_ODR_Addr, n) // 输出
#define PGin(n) BIT_ADDR(GPIOG_IDR_Addr, n)  // 输入

#define PHout(n) BIT_ADDR(GPIOH_ODR_Addr, n) // 输出
#define PHin(n) BIT_ADDR(GPIOH_IDR_Addr, n)  // 输入

#define PIout(n) BIT_ADDR(GPIOI_ODR_Addr, n) // 输出
#define PIin(n) BIT_ADDR(GPIOI_IDR_Addr, n)  // 输入

#define LENARRAY(A) sizeof(A) / sizeof(A[0])
#ifndef ABS
#define ABS(a) ((a) < (0) ? (-(a)) : ((a)))// 取绝对值
#endif

#define ANGLE_RADIAN (0.01745329251994329576922222222222)//(PI / 180.0) // 角度转弧度
#define RADIAN_ANGLE (57.295779513082320876798154814105)//(180.0 / PI)
#define PI 3.14159265358979323846

#define rad2rpm(a) ((a) * 9.5492965855137201461330258023509)//((a)/(PI) * 30.0) //((a)/(2 * PI) * 60)
#define rpm2rad(a) ((a) * 0.10471975511965977461542144610932)//(a * PI / 30.0)

#ifndef SRAM_X
#define SRAM_X 0 
#endif
#ifndef MALLOC
#define MALLOC(type,n)  (type *)mymalloc(SRAM_X,(n)*sizeof(type))
#endif
#ifndef FREE
#define FREE(n) myfree(SRAM_X,n)
#endif
#ifndef REALLOC
#define REALLOC(ptr, type, n) (type *)myrealloc(SRAM_X,ptr, (n) * sizeof(type))
#endif

// 创建一个宏来覆盖标准的printf
#define printf my_printf
#define sqrt fast_sqrt

// 定义一个结构体来存储点的坐标
typedef struct {
    double x;
    double y;
} Point; 

// 定义一个结构体来保存上下文信息
typedef struct {
    uint32_t R0;
    uint32_t R1;
    uint32_t R2;
    uint32_t R3;
    uint32_t R12;
    uint32_t LR;
    uint32_t PC;
    uint32_t PSR;
} HardFaultStackFrame;


typedef uint16_t uint16;
typedef uint8_t uint8;

void NVIC_Configuration(void);
char *my_gcvt(double value, int ndigit, char *buf);
long int time_difference(long int last_second, int last_millisencond, long int second_count, int millisencond_count, int arr);
uint8_t CHeck(uint8_t *data,int number);
// 函数声明
int findParabola(Point p1, Point p2, Point p3, double *a, double *b, double *c);
void print_debug(const char *format, ...);
int my_printf(const char *format, ...);
uint16 crc16_calculate_modbus(uint8 *p_data, uint16 len);
double fast_sqrt(double ptr);
void hard_fault_handler_c(HardFaultStackFrame *psp,HardFaultStackFrame *msp,const char* str);
void car_stop(void);

#endif
