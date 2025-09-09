#ifndef __Z_KINEMATICS__
#define __Z_KINEMATICS__
typedef struct {
	float L0;
	float L1;
	float L2;
	float L3;
	
	float servo_angle[3];	//0到2号舵机角度
	float servo_range[3];		//舵机角度范围
	float servo_pwm[3];		//0到2号舵机角度
}kinematics_t;

void setup_kinematics(float L0, float L1, float L2, float L3, kinematics_t *kinematics);
int kinematics_analysis(float x, float y, float z, float Alpha, kinematics_t *kinematics);
//x, y, z：目标点在空间中的三维坐标（通常以机械臂基座为原点）。
//Alpha：末端执行器的姿态角（如旋转角度，单位通常为弧度）。
//kinematics_t *kinematics：指向已初始化的运动学参数结构体，包含机械臂的尺寸信息；同时也可能用于存储计算出的关节角度结果。

#endif
