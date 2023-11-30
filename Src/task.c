#include "task.h"
#include "tim.h"
#include "imu.h"
#include "odrive.h"
#include "servo.h"
#include "upper.h"
#include "math.h"
#define  FS 1

#if FS==1
	float fast_rate=8.0f,slow_rate=2.0f,mid_rate=5.0f;
#else  
	float fast_rate=12.0f,slow_rate=5.0f,mid_rate=6.50f;
#endif

#define fly_wheel_rate_limit 65 //动量轮速度限幅
#define dt 0.100f
#define PI 3.1415926f
paramTypeDef param;
float PWM_X,PWM_accel,PWM_Final;// PWM中间量
int Steer_Target=0;//舵机pid计算目标值
int Steer_Target_Last=0;//舵机last值
int Steer_Balance = 0;//舵机平衡目标值
int Steer_Balance_Last = 0;//舵机平衡last值
extern int key_times;
int cnt;//角度环计数
int cnt1;//速度环计数
int cnt_vel_callback1;//飞轮速度反馈计数
int cnt_vel_set1;//飞轮速度发送计数
int cnt_balance;//自行车平衡控制周期计数
int cnt_servo;//舵机控制周期计数
int cnt_zero;//动态零点调整周期计数
int cnt_send;//发送上位机
int cnt_rate;//速度设置计数
float zero_det;//动态零点变化量
float rate;//死区外飞轮速度
float Set_steer;//舵机pid目标打角（PWM）
int in_flag=0;//避障开始积分
float start_yaw0;//开始积分时的偏航角
float d_in_k=0.070;//积分比例系数，机械结构减速比
float det_x=0.0f,det_y=0.0f;//m
float last_rate=0;//记录上一时刻的速度
float distance;//停车积分距离
float zer01=0.02;
float zer02=0.001;
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
		if(cnt_rate>=50)
		{
			cnt_rate=0;
			rate_set();//速度设置
		}
		if(cnt_servo>=5)//舵机控制周期 10ms
		{

			Steer_Target = Set_steer=Steer_Engine_control(delta_x_buf);	//舵机打角pid
			Steer_Target = Steer_Speed_Limit(Steer_Target,Steer_Target_Last,1,5); 
			servo_set_duty(Steer_Target);																																						 // 舵机控制
			Steer_Target_Last = Steer_Target;	
			cnt_servo=0;
		}
		if(in_flag==1)
		{
			
				 if(Distance_integral()==1)
					{ 
						 cnt_send++;
						 back_center_send();
					 if(cnt_send>=5)//每次发送一个字节，一共发送五次
					 {
					   in_flag=0;
						 cnt_send=0;
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
				    odrive_speed_ctrl(1,-odrive.set_speed1);
				}
		}
		if(cnt_zero >=	70) //零点变化周期 40ms
		{		
			test_zero_pid();
			cnt_zero = 0;		
		}	
	}
}
/*
函数名称：rate_set
函数功能：分段设置速度
*/
void rate_set()
{
	if(param.run_flag==1)//运行后轮
	{
		if(in_flag==1)//避障参数
		{
			#if FS==1
 			  if(my_abs(delta_x_buf)<=15.0f)odrive.set_speed1=fast_rate*0.005f+last_rate*0.995f;//加速
				else if(my_abs(delta_x_buf)>15.0f&&my_abs(delta_x_buf)<=30.0f)odrive.set_speed1=mid_rate*0.05f+last_rate*0.95f;
				else if(my_abs(delta_x_buf)>30.0f)odrive.set_speed1=slow_rate*0.01f+last_rate*0.99f;
			#else 
			 if(my_abs(delta_x_buf)<=10.0f)odrive.set_speed1=fast_rate*0.005f+last_rate*0.995f;//加速
				else if(my_abs(delta_x_buf)>10.0f&&my_abs(delta_x_buf)<=20.0f)odrive.set_speed1=mid_rate*0.05f+last_rate*0.95f;
				else if(my_abs(delta_x_buf)>20.0f)odrive.set_speed1=slow_rate*0.05f+last_rate*0.95f;
			#endif
		}
		else//直道和弯道参数
		{		
			if(low_speed_flag==0)
			{
				if(my_abs(delta_x_buf)<=15.0f)odrive.set_speed1=fast_rate*0.05f+last_rate*0.95f;//加速
				else if(my_abs(delta_x_buf)>15.0f&&my_abs(delta_x_buf)<=30.0f)odrive.set_speed1=mid_rate*0.01f+last_rate*0.99f;
				else if(my_abs(delta_x_buf)>30.0f)odrive.set_speed1=slow_rate*0.08f+last_rate*0.92f;
			}
			///////////////////完美停车，禁止修改//////////////////
			else if(low_speed_flag==1)//第一次遇到黄线减速
			{
				odrive.set_speed1=1*0.01f+last_rate*0.99f;
				distance-=odrive.now_speed1*dt*d_in_k;
				if(my_fabs(distance)>=2.0f)
				{
					low_speed_flag=0;
					distance=0;
				}
			}else if(low_speed_flag==2)
			{
				odrive.set_speed1=0.14f+last_rate*0.9f;
				distance-=odrive.now_speed1*dt*d_in_k;
				if(my_fabs(distance)>=0.5f)
				{
					param.run_flag=0;
				}
			}
		}
	}
	else
	{
		odrive.set_speed1=0;
	}
	last_rate=odrive.set_speed1;
}
/*
函数名称：Zero_pid_Control
函数功能：零点pid
函数输入：encoder ：当前速度  target_encoder:目标速度
函数返回：零点变化量
*/
float encoder_integral=0;
float Zero_pid_Control(float encoder,float target_encoder)
{

    float encoder_bias,Velocity,rol_bais;
    static float error,last_error,ll;
    error=encoder_bias = encoder - target_encoder;
    encoder_integral = encoder_bias+last_error+ll;
    if(encoder_integral > 1000) 
			encoder_integral = 1000;                 
    if(encoder_integral < -1000) 
			encoder_integral = -1000;                 
		rol_bais=imu.rol-param.angular_zero;
		
//		//分段pid
		if(my_fabs(rol_bais)>=0.3f)
			param.zero_speed_kp=zer01;
		else if(my_fabs(rol_bais)>=0.1f)
			param.zero_speed_kp=zer02;
		else if(my_fabs(rol_bais)<=0.1f)
				param.zero_speed_kp=0;
		
    Velocity =  rol_bais *(param.zero_speed_kp/10)+ encoder_integral/1000 * param.zero_speed_ki+param.zero_speed_kd*(error-last_error);
		ll=last_error;
		last_error=error;
		
    return Velocity;
}
/*
函数名称：test_zero_pid
函数功能：改变动态零点
*/
void test_zero_pid()
{
	
	if(odrive.set_speed0>0.5f)rate=odrive.set_speed0-0.5f;
	else if(odrive.set_speed0<-0.5f)rate=odrive.set_speed0+0.5f;
	zero_det = Zero_pid_Control(rate,0);
	param.angular_zero =param.angular_zero+ zero_det;
}
/*
函数名称：Distance_integral（）
函数功能：路程积分函数
函数返回：积分完成返回1
*/

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
			det_x+=(-odrive.now_speed1)*0.002f*d_in_k*sinf((imu.yaw-start_yaw0)/180*3.14159f);
			det_y+=(-odrive.now_speed1)*0.002f*d_in_k*cosf((imu.yaw-start_yaw0)/180*3.14159f);
			#if FS==1 
			  int ds=3;
			#else 
			  float ds=3.5f;
			#endif
			if(det_y>=ds)//3.0
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
    param.angular_kp = -9.05;
    param.angular_ki = 0;
    param.angular_kd = -5.8;
	
    param.angular_v_kp = -2;
    param.angular_v_ki = 0;
    param.angular_v_kd = -0.985;
	
    param.fly_wheel_speed_kp = 0.99;
    param.fly_wheel_speed_ki = 0;
    param.fly_wheel_speed_kd = 0;
	
	  param.zero_speed_kp=0.0038;
	  param.zero_speed_kd=0;
	  param.zero_speed_ki=0.33;
	  
	
    param.angular_zero = -1;

    param.scope_flag = 0;
		param.run_flag=0;
		
    param.Steer_Kp = 2.5;
    param.Steer_Ki = 0.5;//预防死区
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
    if (bias_intergral >= 10)
        bias_intergral = 10;
    if (bias_intergral <= -10)
        bias_intergral = -10;
    steer_out = param.Steer_Kp * image_bias + param.Steer_Ki * bias_intergral + param.Steer_Kd * (image_bias - Last_image_bias);
    Last_image_bias = image_bias;
    return steer_out;
}
void balance(void)
{
    cnt++;																																																	 // 角度控制周期
    cnt1++;																																																	 // 速度控制周期
																																																						 // 动量轮控制，串级
    if(cnt1>=70){PWM_accel = Velocity_Control(odrive.now_speed0 , 0);cnt1=0;}                                // 动量轮电机速度环正反馈 速度左正右负
    if(cnt>=15){PWM_X = X_balance_Control(imu.rol,param.angular_zero+PWM_accel,imu.vx);cnt=0;}	             // 动量轮电机控制左右倾角param.angular_zero+  角度左负右正
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
			param.run_flag=0;
			key_times=0;
			last_rate=0;
			low_speed_flag=0;
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

