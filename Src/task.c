#include "task.h"
#include "tim.h"
#include "imu.h"
#include "odrive.h"
#include "servo.h"
#include "upper.h"
#include "math.h"
#define fly_wheel_rate_limit 55 //动量轮速度限幅
#define dt 0.002f
#define PI 3.1415926f
paramTypeDef param;
enum bike_state b_s=BALANCE;
extern imu_t imu;
float PWM_X,PWM_accel,PWM_Final;// PWM中间量
int Steer_Target=0;//舵机pid计算目标值
int Steer_Target_Last=0;//舵机last值
int Steer_Balance = 0;//舵机平衡目标值
int Steer_Balance_Last = 0;//舵机平衡last值

int cnt;//角度环计数
int cnt1;//速度环计数
int cnt_vel_callback1;//飞轮速度反馈计数
int cnt_vel_set1;//飞轮速度发送计数
int cnt_balance;//自行车平衡控制周期计数
int cnt_servo;//舵机控制周期计数
int cnt_zero;//动态零点调整周期计数
int cnt_ci;//发送上位机
int cnt_rate;
float zero_det;//动态零点变化量
float rate;//死区外飞轮速度
float Set_steer;//舵机pid目标打角（PWM）
int upper_flag=0;
int in_flag=0;
float start_yaw0;//开始积分时的偏航角
float d_in_k=0.025f;//比例系数
float det_x=0.0f,det_y=0.0f;//m
float fast_rate=8.0f,slow_rate=2.0f;
int run_flag=0;//启动后轮标志位
float last_rate=0;//记录上一时刻的速度
float distance;//停车积分距离
float st_yaw;//开始停车积分的yaw角度
int Distance_integral(void);
//定时器 2ms
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim3)
	{
		imu_get();//陀螺仪读取
		
		cnt_vel_set1++;
    cnt_balance++;	
		cnt_zero++;
		cnt_servo++;

		cnt_rate++;
		if(cnt_rate>=20)
		{
			cnt_rate=0;
			rate_set();//速度设置
		}
		if(cnt_servo>=5)//舵机控制周期 10ms
		{

			Steer_Target = Set_steer=Steer_Engine_control(delta_x_buf);	//舵机打角pid
			Steer_Target = Steer_Speed_Limit(Steer_Target,Steer_Target_Last,1,10); 																 	 // 舵机打角限速31，防止打角太快，车摔倒
			servo_set_duty(Steer_Target);																																						 // 舵机控制
			Steer_Target_Last = Steer_Target;	
			cnt_servo=0;
		}
		if(in_flag==1)
		{
			
				 if(Distance_integral()==1)
					{ 
						 cnt_ci++;
						 back_center_send();
					 if(cnt_ci>=5)//每次发送一个字节，一共发送五次
					 {
					   in_flag=0;
						 cnt_ci=0;
						 det_x=det_y=0;
					 }
				 }
		
    }	
	 
		if(cnt_balance>=1	&& param.scope_flag == 1)//飞轮平衡控制周期 2ms
		{
				balance();
				cnt_balance=0;
		}
		if(cnt_vel_set1 >= 1)//odrive can通信周期 2ms   
		{
						
				cnt_vel_callback1++;
		  	odrive_speed_ctrl(0,odrive.set_speed0);
				
				cnt_vel_set1 = 0;
				if(cnt_vel_callback1 == 20) 
				{
						cnt_vel_callback1 = 0;
				    odrive_speed_ctrl(1,odrive.set_speed1);
				}
			
		}
		if(cnt_zero >=	200) //零点变化周期 400ms
		{		
			test_zero_pid();
			cnt_zero = 0;
			
		}
		
	}
//	else if(htim == &htim4)
//	{
//		cnt_vel_set1++;
//		
//	}
}
//并级pid之飞轮速度环
int k1[6]={1,2,4,5,4,2};
int k2[6]={1,2,4,10,4,5};
void rate_set()
{
	if(run_flag==1)//运行后轮
	{
		if(low_speed_flag==0)
		{
			if(my_abs(delta_x_buf)<=20.0f)odrive.set_speed1=fast_rate*0.01f+last_rate*0.99f;//加速
			else if(my_abs(delta_x_buf)>20.0f)odrive.set_speed1=slow_rate*0.3f+last_rate*0.7f;
		}
		else if(low_speed_flag==1)//第一次遇到黄线减速
		{
			odrive.set_speed1=4*0.1f+last_rate*0.9f;
		}
		else if(low_speed_flag==2)//第二次开环
		{
		
			distance+=odrive.now_speed1*dt*d_in_k*cosf((imu.yaw-st_yaw)/180*PI);
			if(my_fabs(distance*100-error_y)<=15.0f)
			{
				run_flag=0;
			}
		}
		last_rate=odrive.set_speed1;
	}
	else
	{
		odrive.set_speed1=0;
	}
}
float Zero_pid_Control(int encoder,int target_encoder)
{
	float k0;
	if(Steer_Target_Last==Steer_Target)//
	{
		//分段pid的p参数
		if(my_abs(Steer_Target)<=10)k0=k1[0];
		else if(my_abs(Steer_Target)>10&&my_abs(Steer_Target)<=30)k0=k1[1];
		else if(my_abs(Steer_Target)>30&&my_abs(Steer_Target)<=50)k0=k1[2];
		else if(my_abs(Steer_Target)>50&&my_abs(Steer_Target)<=100)k0=k1[3];
		else if(my_abs(Steer_Target)>100&&my_abs(Steer_Target)<=150)k0=k1[4];
		else if(my_abs(Steer_Target)>150) k0=k1[5];
	}
	else 
	{
		if(my_abs(Steer_Target)<=10)k0=k2[0];
		else if(my_abs(Steer_Target)>10&&my_abs(Steer_Target)<=30)k0=k2[1];
		else if(my_abs(Steer_Target)>30&&my_abs(Steer_Target)<=50)k0=k2[2];
		else if(my_abs(Steer_Target)>50&&my_abs(Steer_Target)<=100)k0=k2[3];
		else if(my_abs(Steer_Target)>100&&my_abs(Steer_Target)<=150)k0=k2[4];
		else if(my_abs(Steer_Target)>150) k0=k2[5];
	}

    float encoder_bias,Velocity;
    static float encoder_integral,error,last_error;
    error=encoder_bias = encoder - target_encoder;
    encoder_integral += encoder_bias;
    if(encoder_integral > +0.01) 
			encoder_integral = +0.01;                 
    if(encoder_integral < -0.01) 
			encoder_integral = -0.01;                 
    Velocity = encoder_bias *k0*(param.zero_speed_kp/10 )+ encoder_integral * param.zero_speed_ki+param.zero_speed_kd*(error-last_error);
		last_error=error;
    return Velocity;
}

void test_zero_pid()
{
	
	if(odrive.set_speed0>2)rate=odrive.set_speed0-2;
	else if(odrive.set_speed0<-2)rate=odrive.set_speed0+2;
	zero_det = Zero_pid_Control(rate,0);
	param.angular_zero += zero_det;
}
//路程积分函数

int Distance_integral()
{
	static int state = 0;
	switch(state)
	{
		case 0:
		{
			start_yaw0=imu.yaw;
			state=1;
			break;
		}
		case 1:
		{
			det_x+=odrive.now_speed1*dt*d_in_k*sinf((imu.yaw-start_yaw0)/180*3.14159f);
			det_y+=odrive.now_speed1*dt*d_in_k*cosf((imu.yaw-start_yaw0)/180*3.14159f);
			if(det_y>=3)
			{
				state=0;
				return 1;
			}
			break;
	  }
		default:break;
	}
	return 0;
}
//pid参数初始化
void param_init(){
    param.angular_kp = -10.45;
    param.angular_ki = 0;
    param.angular_kd = -5.8;
	
    param.angular_v_kp = -3.5;
    param.angular_v_ki = 0;
    param.angular_v_kd = -0.96;
	
    param.fly_wheel_speed_kp = 0.68;
    param.fly_wheel_speed_ki = 0;
    param.fly_wheel_speed_kd = 0;
	
	  param.zero_speed_kp=0.0025;
	  param.zero_speed_kd=0.001;
	  param.zero_speed_ki=0;
	  
		param.zero_steer_kp=0;
		param.zero_steer_ki=0;
	  param.zero_steer_kd=0;
	
		param.zero_accl_kp=0;
		param.zero_accl_ki=0;
		param.zero_accl_kd=0;
	
    param.angular_zero = -1;

    param.scope_flag = 0;
		
    param.Steer_Kp = 3;
    param.Steer_Ki = 0;
    param.Steer_Kd = 0;

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
     static float error;
     Bias=Angle-Angle_Zero;                                            //获取偏差
     error+=Bias;                                                      //偏差累积
     if(error>+30) error=+30;                                          //积分限幅
     if(error<-30) error=-30;                                          //积分限幅
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
//舵机pid
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

																																																						 // 动量轮控制，串级
    if(cnt1>=60){PWM_accel = Velocity_Control(odrive.now_speed0 , 0);cnt1=0;}     // 动量轮电机速度环正反馈 速度左正右负
    if(cnt>=15){PWM_X = X_balance_Control(imu.rol,param.angular_zero+PWM_accel,imu.vx);cnt=0;}	 // 动量轮电机控制左右倾角param.angular_zero+  角度左负右正
    PWM_Final = Angle_Velocity(imu.vx,PWM_X);       																												 // 角速度环  角速度左负右正
    odrive.set_speed0 = PWM_Final;																																					 // 速度设置左正右负     
			  																																																		 // 动量轮限幅
    if(odrive.set_speed0>fly_wheel_rate_limit) odrive.set_speed0=fly_wheel_rate_limit;      								 // 动量轮电机限幅
    else if(odrive.set_speed0<-fly_wheel_rate_limit) odrive.set_speed0=-fly_wheel_rate_limit; 							 // 动量轮电机限幅
		
																																																						 // 摔倒停车判,断
    if((imu.rol-(param.angular_zero))>3 || (imu.rol-(param.angular_zero))<-3)
		{
			param.scope_flag=0;
			odrive.set_speed0=odrive.set_speed1=0;
			param.angular_zero=-1.1;
			run_flag=0;
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

