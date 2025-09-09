#include "encoder.h"



u8 encoder_tim_psc = 1-1; //默认不分频


double encoder_Num[4] = {0}; // 电机1 编码器值


double encoder_Num1_raw = 0; // 电机1 编码器值
double encoder_Num2_raw = 0;
double encoder_Num3_raw = 0;
double encoder_Num4_raw = 0;

__attribute__((noinline)) void TIMx_Encoder_psc_reinit(TIM_TypeDef* TIMx)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
  TIM_ICInitTypeDef TIM_ICInitStruct;

  TIM_DeInit(TIMx);
  TIM_TimeBaseStructInit(&TIM_TimeBaseStruct);
  TIM_TimeBaseStruct.TIM_Prescaler = encoder_tim_psc;
  TIM_TimeBaseStruct.TIM_Period = ENCODER_TIM_PERIOD;
  TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStruct);

  TIM_EncoderInterfaceConfig(TIMx, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
  TIM_ICStructInit(&TIM_ICInitStruct);
  TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV4;
  TIM_ICInitStruct.TIM_ICFilter = 0x9; // 0x0~0xF数越大滤波效果越好
  TIM_ICInit(TIMx, &TIM_ICInitStruct);
  TIM_SetCounter(TIMx, CNT_INIT);
  TIM_ClearFlag(TIMx, TIM_IT_Update);
  TIM_ITConfig(TIMx, TIM_IT_Update, DISABLE);
  TIM_Cmd(TIMx, ENABLE);
}


// PA0 PA1-M1 电机1 右前
__attribute__((noinline)) void TIM5_Encoder_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
  TIM_ICInitTypeDef TIM_ICInitStruct;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;// 推挽输出

  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉

  GPIO_Init(GPIOA, &GPIO_InitStructure);

  TIM_DeInit(TIM5);
  TIM_TimeBaseStructInit(&TIM_TimeBaseStruct);
  TIM_TimeBaseStruct.TIM_Prescaler = encoder_tim_psc;
  TIM_TimeBaseStruct.TIM_Period = ENCODER_TIM_PERIOD;
  TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStruct);

  TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
  TIM_ICStructInit(&TIM_ICInitStruct);
  TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV4;
  TIM_ICInitStruct.TIM_ICFilter = 0x9; // 0x0~0xF数越大滤波效果越好
  TIM_ICInit(TIM5, &TIM_ICInitStruct);
  TIM_SetCounter(TIM5, CNT_INIT);
  TIM_ClearFlag(TIM5, TIM_IT_Update);
  TIM_ITConfig(TIM5, TIM_IT_Update, DISABLE);
  TIM_Cmd(TIM5, ENABLE);
}

// PD12 PD13-M3 右后
__attribute__((noinline)) void TIM4_Encoder_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
  TIM_ICInitTypeDef TIM_ICInitStruct;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);

  GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

  GPIO_Init(GPIOD, &GPIO_InitStructure);

  TIM_DeInit(TIM4);
  TIM_TimeBaseStructInit(&TIM_TimeBaseStruct);
  TIM_TimeBaseStruct.TIM_Prescaler = encoder_tim_psc;
  TIM_TimeBaseStruct.TIM_Period = ENCODER_TIM_PERIOD;
  TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStruct);

  TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
  TIM_ICStructInit(&TIM_ICInitStruct);
  TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV4;
  TIM_ICInitStruct.TIM_ICFilter = 0x9; // 0x0~0xF数越大滤波效果越好
  TIM_ICInit(TIM4, &TIM_ICInitStruct);
  TIM_SetCounter(TIM4, CNT_INIT);
  TIM_ClearFlag(TIM4, TIM_IT_Update);
  TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
  TIM_Cmd(TIM4, ENABLE);
}

// PB4 PB5-M4 左后
__attribute__((noinline)) void TIM3_Encoder_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
  TIM_ICInitTypeDef TIM_ICInitStruct;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

  GPIO_Init(GPIOB, &GPIO_InitStructure);

  TIM_DeInit(TIM3);
  TIM_TimeBaseStructInit(&TIM_TimeBaseStruct);
  TIM_TimeBaseStruct.TIM_Prescaler = encoder_tim_psc;
  TIM_TimeBaseStruct.TIM_Period = ENCODER_TIM_PERIOD;
  TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStruct);

  TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
  TIM_ICStructInit(&TIM_ICInitStruct);
  TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV4;
  TIM_ICInitStruct.TIM_ICFilter = 0x9; // 0x0~0xF数越大滤波效果越好
  TIM_ICInit(TIM3, &TIM_ICInitStruct);
  TIM_SetCounter(TIM3, CNT_INIT);
  TIM_ClearFlag(TIM3, TIM_IT_Update);
  TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
  TIM_Cmd(TIM3, ENABLE);
}
// PB3 PA15-M2 左前
__attribute__((noinline)) void TIM2_Encoder_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
  TIM_ICInitTypeDef TIM_ICInitStruct;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_TIM2);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_TIM2);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

  GPIO_Init(GPIOA, &GPIO_InitStructure);

  TIM_DeInit(TIM2);

  TIM_TimeBaseStruct.TIM_Prescaler = encoder_tim_psc;
  TIM_TimeBaseStruct.TIM_Period = ENCODER_TIM_PERIOD;
  TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStruct);

  TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
  TIM_ICStructInit(&TIM_ICInitStruct);
  TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV4;
  TIM_ICInitStruct.TIM_ICFilter = 0x9; // 0x0~0xF数越大滤波效果越好
  TIM_ICInit(TIM2, &TIM_ICInitStruct);
  TIM_SetCounter(TIM2, CNT_INIT);
  TIM_ClearFlag(TIM2, TIM_IT_Update);
  TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
  TIM_Cmd(TIM2, ENABLE);
}



int encoderNum = 0;
inline int read_encoder_tim2(void)
{

  encoderNum = (int)((int16_t)(TIM2->CNT));
  TIM2->CNT = CNT_INIT;
  // TIM_SetCounter(TIM2, CNT_INIT);

  return encoderNum;
}

inline int read_encoder_tim3(void)
{

  encoderNum = (int)((int16_t)(TIM3->CNT));
  TIM3->CNT = CNT_INIT;
  // TIM_SetCounter(TIM3, CNT_INIT);

  return encoderNum;
}

inline int read_encoder_tim4(void)
{

  encoderNum = (int)((int16_t)(TIM4->CNT));
  TIM4->CNT = CNT_INIT;
  // TIM_SetCounter(TIM4, CNT_INIT);

  return encoderNum;
}

inline int read_encoder_tim5(void)
{

  encoderNum = (int)((int16_t)(TIM5->CNT));
  TIM5->CNT = CNT_INIT;
  // TIM_SetCounter(TIM5, CNT_INIT);

  return encoderNum;
}

  // double encoderNum_tim2 = 0;
  // double encoderNum_tim3 = 0;
  // double encoderNum_tim4 = 0;
  // double encoderNum_tim5 = 0;

inline void calc_motor_rotate_speed()
{

  // encoderNum_tim2 = -read_encoder_tim2();
  encoder_Num2_raw = (-read_encoder_tim2());
  encoder_Num[1] = readSpeed(encoder_Num2_raw); // 编码器速度传递给speed_Tyer

  // encoderNum_tim3 = read_encoder_tim3(); // ----compare1
  encoder_Num4_raw = (read_encoder_tim3());
  encoder_Num[3] = readSpeed(encoder_Num4_raw);

  // encoderNum_tim4 = read_encoder_tim4();     // ----compare2
  encoder_Num3_raw = (read_encoder_tim4());
  encoder_Num[2] = readSpeed(encoder_Num3_raw); // 编码器速度传递给speed_Tyer


  // encoderNum_tim5 = read_encoder_tim5();
  encoder_Num1_raw = (read_encoder_tim5());
  encoder_Num[0] = readSpeed(encoder_Num1_raw);
}