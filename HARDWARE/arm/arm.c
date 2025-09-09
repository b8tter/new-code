#include "x_rcc.h"		
#include "x_gpio.h"		
#include "x_global.h"	
#include "x_delay.h"	
#include "x_type.h"		
#include "x_usart.h"	
#include "x_timer.h"	
#include "x_ps2.h"		
#include "x_w25q64.h"	
#include "x_sensor.h"	
#include "x_adc.h"		
#include <stdio.h>		
#include <string.h>		
#include <math.h>		
#include "x_kinematics.h"	
#define pi 3.1415926

/*
单位1mm
*/

void setup_kinematics(float L0, float L1, float L2,kinematics_t *kinematics) {
	kinematics->L0 = L0*10;
	kinematics->L1 = L1*10;
	kinematics->L2 = L2*10;
    kinematics->L3 = L3*10;
}

int kinematics_analysis(float x, float y, float z, float Alpha, kinematics_t *kinematics) {

	float theta4,theta5, theta6;
	float l0, l1, l2,l3;//L0 L1  L2  L3
	float aaa, bbb,ccc,zf_flag;
	
	//·Å´ó10±¶
	x = x*10;
	y = y*10;
	z = z*10;
	
	
	l0 = kinematics->L0;
	l1 = kinematics->L1;
	l2 = kinematics->L2;
	l3 = kinematics->L3;
    

	
	 if(x == 0){
        theta6 = 0.0;
	 }
    else if(x>0 && y<0){
        theta6 = atan(x / y);
        theta6 = 180 + (theta6 * 180.0/pi);
    }else{
			  if(y>=0){
				   theta6 = atan(x / y);
           theta6 = theta6 * 180.0 / pi;
				}else{
					
			  	theta6 = atan(x / y);
          theta6 = theta6 * 180.0 / pi;
				  theta6=theta6-180;
				}
        
		}
	  y = sqrt(x*x + y*y);
    y = y-l3 * cos(Alpha*pi/180.0);  //Çó³ö y×Ü - y3 = y2 + y1
    z = z-l0-l3*sin(Alpha*pi/180.0); //Çó³öz1 + z2
    if(z < -l0) {
        return 1;
	}
    if(sqrt(y*y + z*z) > (l1+l2)) {
        return 2;
	}
	
  	ccc = acos(y / sqrt(y * y + z * z));
    bbb = (y*y+z*z+l1*l1-l2*l2)/(2*l1*sqrt(y*y+z*z));//ÓàÏÒ¶¨Àí
    if(bbb > 1 || bbb < -1) {
        return 5;
	}
    if (z < 0) {
        zf_flag = -1;
	} else {
        zf_flag = 1;
	}
    theta5 = ccc * zf_flag + acos(bbb);
    theta5 = theta5 * 180.0 / pi;
    if(theta5 > 180.0 || theta5 < 0.0) {
        return 6;
	}
	
    aaa = -(y*y+z*z-l1*l1-l2*l2)/(2*l1*l2);
    if (aaa > 1 || aaa < -1) {
        return 3;
	}
    theta4 = acos(aaa); 
    theta4 = 180.0 - theta4 * 180.0 / pi ;  
    if (theta4 > 135.0 || theta4 < -135.0) {
        return 4;
	}
	}

//  sprintf((char *)cmd_return,"%04d",(int)theta6);
//	zx_uart_send_str(cmd_return);


  kinematics->servo_angle[0] = theta6;
	kinematics->servo_angle[1] = theta5-90;
	kinematics->servo_angle[2] = theta4;
	 
	//»»Ëã¶æ»ú½Ç¶È
	kinematics->servo_pwm[0] = (int)(1500-2000.0 * kinematics->servo_angle[0] / 270.0);
	kinematics->servo_pwm[1] = (int)(1500+2000.0 * kinematics->servo_angle[1] / 270.0);
	kinematics->servo_pwm[2] = (int)(1500+2000.0 * kinematics->servo_angle[2] / 270.0);

	return 0;
}
