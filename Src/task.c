#include "task.h"
#include "tim.h"
#include "imu.h"
#include "odrive.h"
#include "servo.h"

#define fly_wheel_rate_limit 55 //动量轮速度限幅
//
paramTypeDef param;
enum bike_state b_s=BALANCE;
extern imu_t imu;
float PWM_X,PWM_accel,PWM_Final;                  // PWM中间量
int Flag_Stop = 1;

int Steer_Target=0;//舵机pid计算目标值
int Steer_Target_Last=0;//舵机last值
int Steer_Balance = 0;//舵机平衡目标值
int Steer_Balance_Last = 0;//舵机平衡last值

int cnt;//角度环计数
int cnt1;//速度环计数
int cnt_vel_callback;
int cnt_vel_set;
int cnt_balance;

float Roll_Change = 0;//动态零点变化量
float Pitch_Change_Last = 0;//上一次动态零点

int test_servo=0;//舵机打角debug测试
int test_servo_flag=1;
float test_rate=0;

int zero_test_cnt = 0;
float zero_delta = 0;

int demo_steer_target = 0;

void test_zero_pid(void);
void test_zero(void);
//定时器 2ms
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim3)
	{
		imu_get();
//		if(test_servo_flag==1)
//		{
//				servo_set_duty(test_servo);
//		}
		// 这里有控制周期的 这样直接写不知道行不行啊  是不是得有cnt这种的 这个定时器周期和原来代码是一样的
		// task
		cnt_vel_set++;
    cnt_balance++;	
		zero_test_cnt++;
		Steer_Target = demo_steer_target;
		Steer_Target = Steer_Speed_Limit(Steer_Target,Steer_Target_Last,1,1); 																 	 // 舵机打角限速31，防止打角太快，车摔倒
    servo_set_duty(Steer_Target);																																						 // 舵机控制
    Steer_Target_Last = Steer_Target;																																				 // 记录上次打角值
		//odrive_speed_ctrl(1,test_rate);
		if(cnt_balance>=1	&& param.scope_flag == 1)
		{
			
				balance();
				cnt_balance=0;
		}
		if(cnt_vel_set == 1)//2ms   
		{
				odrive_speed_ctrl(0,odrive.set_speed0);
				cnt_vel_callback++;
				cnt_vel_set = 0;
				if(cnt_vel_callback == 2) // 4ms
				{
						cnt_vel_callback = 0;
						odrive_vel_callback();
				}
			
		}
		if(zero_test_cnt >=	50)
		{
			test_zero_pid();
			zero_test_cnt = 0;
		}
		
	}
}
float Zero_pid_Control(int encoder,int target_encoder)
{
    float encoder_bias,Velocity;
    static float encoder_integral,error,last_error;
    error=encoder_bias = encoder - target_encoder;
    encoder_integral += encoder_bias;
    if(encoder_integral > +0.001) 
			encoder_integral = +0.001;                    //积分限幅
    if(encoder_integral < -0.001) 
			encoder_integral = -0.001;                    //积分限幅是500
    Velocity = encoder_bias *param.zero_speed_kp/10 + encoder_integral * param.zero_speed_ki+param.zero_speed_kd*(error-last_error);
		last_error=error;
    return Velocity;
}
float zero_det;
float rate;
void test_zero_pid()
{
	
	if(odrive.set_speed0>3)rate=odrive.set_speed0-3;
	else if(odrive.set_speed0<-3)rate=odrive.set_speed0+3;
	zero_det = Zero_pid_Control(rate , param.fly_whell_speed_target);
	param.angular_zero += zero_det;
}
void test_zero()
{
	static int time_flag = 0;
	static int time_clear_delta = 0;
	if(odrive.now_speed0>2)
	{
		if(time_flag < 0)
			time_flag = 0;
		time_flag ++;
		if(time_flag >= 10)
		{
			zero_delta += odrive.now_speed0 * 0.0001f;
			if(time_clear_delta < 0)
				time_clear_delta = 0;
			time_clear_delta ++;
			if(time_clear_delta >= 2)
			{
				zero_delta = 0;
				time_clear_delta = 0;
			}
				
		}
	}
	else if(odrive.now_speed0<-2)
	{
		if(time_flag > 0)
			time_flag = 0;
		time_flag --;
		if(time_flag <= -10)
		{
			zero_delta -= odrive.now_speed0 * 0.0001f;
			if(time_clear_delta > 0)
				time_clear_delta = 0;
			time_clear_delta --;
			if(time_clear_delta <= -2)
			{
				zero_delta = 0;
				time_clear_delta = 0;
			}
		}
	}
	else
	{
		zero_delta = 0;
		time_flag = 0;
	}
	param.angular_zero +=zero_delta;
}
//pid参数初始化
void param_init(){
    param.angular_kp = -10.45;//并级 -32.05 0 -6.205       12 5 0
    param.angular_ki = 0;
    param.angular_kd = -5.8;
	
    param.angular_v_kp = -1.85;
    param.angular_v_ki = 0;
    param.angular_v_kd = -0.86;
	
    param.fly_wheel_speed_kp = 0.68;
    param.fly_wheel_speed_ki = 0;
    param.fly_wheel_speed_kd = 0;
	
	  param.zero_speed_kp=0.01;
	  param.zero_speed_kd=0.0001;
	  param.zero_speed_ki=0;
	  
	
    param.angular_zero = -2;
    param.fly_whell_speed_target = 0;
    param.scope_flag = 0;
    param.Steer_Kp = 1;//舵机kp
    param.Steer_Ki = 0;//舵机kp
    param.Steer_Kd = 0;//舵机kd
    param.Balance_Kp = 0;
    param.Balance_Ki = 0;
    param.Balance_Kd = 0;

}
//角速度环pid
float Angle_Velocity(float Gyro,float Gyro_Target)
{
    float Angle_Velocity_Bias;
    float PWM_Out;
    static float Angle_Velocity_Last_Bias,Angle_Velocity_Integral;
    Angle_Velocity_Bias = Gyro - Gyro_Target;
    Angle_Velocity_Integral+=Angle_Velocity_Bias;
    if(Angle_Velocity_Integral > 10000)
			Angle_Velocity_Integral =10000;                   
    if(Angle_Velocity_Integral < -10000) 
			Angle_Velocity_Integral = -10000;        
		
    PWM_Out = param.angular_v_kp * Angle_Velocity_Bias + param.angular_v_ki * Angle_Velocity_Integral + param.angular_v_kd * (Angle_Velocity_Bias - Angle_Velocity_Last_Bias);
    Angle_Velocity_Last_Bias = Angle_Velocity_Bias;                             //保留上次误差
    return PWM_Out;
}
//角度环pid
float X_balance_Control(float Angle,float Angle_Zero,float gyro)
{
     float PWM,Bias;
     static float error;//,X_Balance_Last_Bias;
     Bias=Angle-Angle_Zero;                                            //获取偏差
     error+=Bias;                                                      //偏差累积
     if(error>+30) error=+30;                                          //积分限幅
     if(error<-30) error=-30;                                          //积分限幅
    // PWM=param.angular_kp*Bias + param.angular_ki*error + (Bias - X_Balance_Last_Bias)*param.angular_kd;   //获取最终数值
     PWM=param.angular_kp*Bias + param.angular_ki*error + (gyro)*param.angular_kd;   //获取最终数值
     return PWM;
}
//速度环pid
float Velocity_Control(int encoder,int target_encoder)
{
    float encoder_bias,Velocity;
    static float encoder_bias_integral;
    encoder_bias = encoder - target_encoder;
    encoder_bias_integral += encoder_bias;
    if(encoder_bias_integral > +200) 
			encoder_bias_integral = +200;                    //积分限幅
    if(encoder_bias_integral < -200) 
			encoder_bias_integral = -200;                    //积分限幅是500
    Velocity = encoder_bias * param.fly_wheel_speed_kp/10 + encoder_bias_integral * param.fly_wheel_speed_ki/1000;
    return Velocity;
}
int Steer_Engine_control(float image_bias)
{
    int steer_out;
    static float Last_image_bias;
    static float bias_intergral;
    bias_intergral += image_bias;
    if (bias_intergral >= 50)
        bias_intergral = 50;
    if (bias_intergral <= -50)
        bias_intergral = -50;
    steer_out = param.Steer_Kp * image_bias + param.Steer_Ki * bias_intergral + param.Steer_Kd * (image_bias - Last_image_bias);
    Last_image_bias = image_bias;
    return steer_out;
}
void balance(void)
{
    cnt++;																																																	 // 角度控制周期
    cnt1++;																																																	 // 速度控制周期

		//Roll_Change = Roll_Change_PD(Steer_Target,0);																													 // 动态零点变化量  
																																																						 // 动量轮控制，串级
    if(cnt1>=60){PWM_accel = Velocity_Control(odrive.now_speed0 , param.fly_whell_speed_target);cnt1=0;}     // 动量轮电机速度环正反馈 速度左正右负
    if(cnt>=15){PWM_X = X_balance_Control(imu.rol,param.angular_zero+PWM_accel+Roll_Change,imu.vx);cnt=0;}	 // 动量轮电机控制左右倾角param.angular_zero+  角度左负右正
    PWM_Final = Angle_Velocity(imu.vx,PWM_X);       																												 // 角速度环  角速度左负右正
    odrive.set_speed0 = PWM_Final;																																					 // 速度设置左正右负     
			  																																																		 // 动量轮限幅
    if(odrive.set_speed0>fly_wheel_rate_limit) odrive.set_speed0=fly_wheel_rate_limit;      								 // 动量轮电机限幅
    else if(odrive.set_speed0<-fly_wheel_rate_limit) odrive.set_speed0=-fly_wheel_rate_limit; 							 // 动量轮电机限幅
		
		 
//    Steer_Balance = SBB_Get_BalancePID(imu.rol,imu.vx,param.angular_zero+PWM_accel+Roll_Change);             // 舵机平衡
//    Steer_Balance = Steer_Speed_Limit(Steer_Balance,Steer_Balance_Last,1,10); 															 // 舵机打角限速
//    Steer_Balance_Last = Steer_Balance;

    

																																																						 // 摔倒停车判,断
    if(b_s != END && ((imu.rol-(param.angular_zero+Roll_Change))>3 || (imu.rol-(param.angular_zero+Roll_Change))<-3))  b_s = STOP;
    if(b_s == BEGINE || b_s == STOP || b_s == END)  odrive.set_speed0 = 0;
    
}
/*
函数名称:Roll_Change_PD
函数功能: 动态平衡零点
*/
float Roll_Change_PD(int steer_angle,int flag)
{
    float roll_change;
    if(flag == 0)//右
    {
       if(steer_angle>0){//左
            if(my_abs(steer_angle)>0 && my_abs(steer_angle)<25) roll_change = 0;
            else if(my_abs(steer_angle)>=25 && my_abs(steer_angle)<50) roll_change = 0.1;
            else if(my_abs(steer_angle)>=50 && my_abs(steer_angle)<75) roll_change = 0.23;
            else if(my_abs(steer_angle)>=75 && my_abs(steer_angle)<100) roll_change = 0.4;
            else if(my_abs(steer_angle)>=100 && my_abs(steer_angle)<125) roll_change = 0.5;
            else if(my_abs(steer_angle)>=125 && my_abs(steer_angle)<150) roll_change = 0.7;
            else if(my_abs(steer_angle)>=150 && my_abs(steer_angle)<175) roll_change = 0.8;
            else if(my_abs(steer_angle)>=175 && my_abs(steer_angle)<200) roll_change = 0.9;
            else roll_change = 1;
            // roll_change += 0.75;
        }
        else if(steer_angle<0){//右
            if(my_abs(steer_angle)>0 && my_abs(steer_angle)<25) roll_change = 0;  
            else if(my_abs(steer_angle)>=25 && my_abs(steer_angle)<50) roll_change = -0.3;////////
            else if(my_abs(steer_angle)>=50 && my_abs(steer_angle)<75) roll_change = -0.47;////////
            else if(my_abs(steer_angle)>=75 && my_abs(steer_angle)<100) roll_change = -0.65;///////
            else if(my_abs(steer_angle)>=100 && my_abs(steer_angle)<125) roll_change = -0.8;//////
            else if(my_abs(steer_angle)>=125 && my_abs(steer_angle)<150) roll_change = -0.95;//////
            else if(my_abs(steer_angle)>=150 && my_abs(steer_angle)<175) roll_change = -1.25;///////
            else if(my_abs(steer_angle)>=175 && my_abs(steer_angle)<200) roll_change = -1.4;/////
            else roll_change = -1.5;//  325  -1.2//////
            // roll_change += 0.5;
        }
        else if(steer_angle == 0) roll_change = 0;
    }
    else if(flag == 1)//左
    {
        if(steer_angle>0){//左
            if(my_abs(steer_angle)>0 && my_abs(steer_angle)<25) roll_change = -0.1;//////////
            else if(my_abs(steer_angle)>=25  && my_abs(steer_angle)<50)  roll_change = 0.2;////////
            else if(my_abs(steer_angle)>=50  && my_abs(steer_angle)<75)  roll_change = 0.6;///////
            else if(my_abs(steer_angle)>=75  && my_abs(steer_angle)<100) roll_change = 1;/////
            else if(my_abs(steer_angle)>=100 && my_abs(steer_angle)<125) roll_change = 1.6;/////
            else if(my_abs(steer_angle)>=125 && my_abs(steer_angle)<150) roll_change = 1.9;///////
            else if(my_abs(steer_angle)>=150 && my_abs(steer_angle)<175) roll_change = 2.1;///////
            else if(my_abs(steer_angle)>=175 && my_abs(steer_angle)<200) roll_change = 2.4;////////
            else roll_change = 2.5;///////  325  -1.2
            roll_change -= 0.5f;
        }
        else if(steer_angle<0){//右
            if(my_abs(steer_angle)>0 && my_abs(steer_angle)<25) roll_change = 0;  
            else if(my_abs(steer_angle)>=25  && my_abs(steer_angle)<50)  roll_change = -0.1;////////
            else if(my_abs(steer_angle)>=50  && my_abs(steer_angle)<75)  roll_change = -0.5;////////
            else if(my_abs(steer_angle)>=75  && my_abs(steer_angle)<100) roll_change = -1.1;///////
            else if(my_abs(steer_angle)>=100 && my_abs(steer_angle)<125) roll_change = -1.5;//////
            else if(my_abs(steer_angle)>=125 && my_abs(steer_angle)<150) roll_change = -2.2;//////
            else if(my_abs(steer_angle)>=150 && my_abs(steer_angle)<175) roll_change = -2.2;///////
            else if(my_abs(steer_angle)>=175 && my_abs(steer_angle)<200) roll_change = -2.4;/////
            else roll_change = -2.6;//  325  -1.2//////
            roll_change -= 0.7f;
        }
        else if(steer_angle == 0) roll_change = 0;
    }
   else if(flag == 2)
    {
        if(steer_angle>0){//-2.05
            if(my_abs(steer_angle)>0 && my_abs(steer_angle)<25) roll_change = 0.1;
            else if(my_abs(steer_angle)>=25  && my_abs(steer_angle)<50) roll_change = 0.2;
            else if(my_abs(steer_angle)>=50  && my_abs(steer_angle)<75) roll_change = 0.6;
            else if(my_abs(steer_angle)>=75  && my_abs(steer_angle)<100) roll_change = 1;
            else if(my_abs(steer_angle)>=100 && my_abs(steer_angle)<125) roll_change = 1.3;
            else if(my_abs(steer_angle)>=125 && my_abs(steer_angle)<150) roll_change = 1.6;
            else if(my_abs(steer_angle)>=150 && my_abs(steer_angle)<175) roll_change = 2;
            else if(my_abs(steer_angle)>=175 && my_abs(steer_angle)<200) roll_change = 2.3;
            else roll_change = 2.5;///////  325  -1.2
        }
        else if(steer_angle<0){//右
            if(my_abs(steer_angle)>0 && my_abs(steer_angle)<25) roll_change = -0.2;
            else if(my_abs(steer_angle)>=25 && my_abs(steer_angle)<50) roll_change = -0.3;////////
            else if(my_abs(steer_angle)>=50 && my_abs(steer_angle)<75) roll_change = -0.7;////////
            else if(my_abs(steer_angle)>=75 && my_abs(steer_angle)<100) roll_change = -1.1;///////
            else if(my_abs(steer_angle)>=100 && my_abs(steer_angle)<125) roll_change = -1.5;//////
            else if(my_abs(steer_angle)>=125 && my_abs(steer_angle)<150) roll_change = -2;//////
            else if(my_abs(steer_angle)>=150 && my_abs(steer_angle)<175) roll_change = -2.2;///////
            else if(my_abs(steer_angle)>=175 && my_abs(steer_angle)<200) roll_change = -2.4;/////
            else roll_change = -2.6;//  325  -1.2//////
        }
        else if(steer_angle == 0) roll_change = 0;
    }
    return roll_change;
}

int SBB_Get_BalancePID(float Angle,float Gyro,float Pitch_Calculate)
{
    float  Bias;
    static float Integration=0;
    int SBB_BalancePID;
    Bias = Angle - Pitch_Calculate;     // 求出平衡的角度中值和此时横滚角的偏差
    Integration += Bias;           // 积分
    if(Integration<-380)      Integration=-380; //限幅
    else if(Integration>380)  Integration= 380; //限幅
    //===计算平衡控制的舵机PWM  PID控制 kp是P系数 ki式I系数 kd是D系数
    if(Bias>=0)
        SBB_BalancePID = -param.Balance_Kp * (Bias*Bias) - param.Balance_Ki*Integration - param.Balance_Kd*Gyro;
    else
        SBB_BalancePID = -param.Balance_Kp * (-Bias*Bias) - param.Balance_Ki*Integration - param.Balance_Kd*Gyro;
    return SBB_BalancePID;

}
void odrive_limit()
{
	if(odrive.set_speed1>0&&my_fabs(odrive.now_speed1)<=0.1f)
	{
		odrive.set_speed1=0;
	}
}
int my_abs(int x)
{
	  float m;
    if(x>=0)
        m= x;
    else if(x<0)
        m= -x;
		return m;
}
float my_fabs(float x)
{
	  float m;
    if(x>=0)
        m= x;
    else if(x<0)
        m= -x;
		return m;
}

