#ifndef __TASK_H__
#define __TASK_H__


#include "main.h"
//pid参数结构体
typedef struct 
{
		//角速度环
    float angular_v_kp;
    float angular_v_ki;
    float angular_v_kd;
		//角度环
    float angular_kp;
    float angular_ki;
    float angular_kd;
		//速度环
    float fly_wheel_speed_kp;
    float fly_wheel_speed_ki;
    float fly_wheel_speed_kd;
	  //零点速度环
	  float zero_speed_kp;
    float zero_speed_ki;
    float zero_speed_kd;
	
    float angular_zero;             //角度零点
    float angular_target;           //目标角度
    float fly_whell_speed_target;   //飞轮速期望
    float scope_flag;
    float Steer_Kp;                 //舵机kp
    float Steer_Ki;                 //舵机ki
    float Steer_Kd;                 //舵机kd
    float Balance_Kp;               //舵机平衡kp
    float Balance_Ki;               //舵机平衡ki
    float Balance_Kd;               //舵机平衡kd
}paramTypeDef;
enum bike_state{BEGINE=0, BALANCE, SEND, RUN, STOP, END, LINE_END};
extern enum bike_state b_s;
int my_abs(int x);
float my_fabs(float x);
int SBB_Get_BalancePID(float Angle,float Gyro,float Pitch_Calculate);
int Steer_Engine_control(float image_bias);
float Roll_Change_PD(int steer_angle,int flag);
void param_init(void);
void balance(void);
#endif

