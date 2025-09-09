#include "HD.h"
#include "system.h"                  
#include "motor.h"                

#ifdef HUI_DU
void HDInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
    GPIO_InitStructure.GPIO_Pin =
        GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
        GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 |
        GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    // GPIO_InitStructure.GPIO_Mode = GPIO_MODE_INPUT;
    // GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    // GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOF, &GPIO_InitStructure);

    
    GPIO_SetBits(GPIOF, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
                            GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 |
                            GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11);
}
#endif

#ifdef motor
void motor(int L_Lun, int R_Lun){
    // �����������ٶ�
    LeftWheelBehind_Speed(L_Lun);   //����������ٶ�
    LeftWheelFront_Speed(L_Lun);    // ������ǰ���ٶ�
    RightWheelBehind_Speed(R_Lun);  // �����Һ����ٶ�
    RightWheelFront_Speed(R_Lun);   // ������ǰ���ٶ�
}
