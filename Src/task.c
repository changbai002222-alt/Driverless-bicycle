#include "task.h"
#include "tim.h" 
#include "imu.h"
#include "odrive.h"
#include "servo.h"
#include "upper.h"
#include "math.h"

extern volatile float Remote_Speed;
extern volatile float Remote_Steer;
extern volatile float Roll_Zero;

#define SERVO_CENTER_PWM 1500  // 舵机零点   

#define RIGHT_RETURN_PWM 1550  // 右转回中值
#define STEP_SIZE        0.005f  
#define BASE_ANGLE_ZERO  -2.35f  // 基础物理零点
#define COMPENSATE_K     -0.004f 
#define SERVO_MAX_PWM    1800  // 舵机左转极限
#define SERVO_MIN_PWM    1200  // 舵机右转极限
#define PROTECT_ANGLE    8.0f   // 倒地保护角度

paramTypeDef param;

float PWM_X,PWM_accel,PWM_Final;
extern int key_times;
int cnt;
int cnt1;
int cnt_vel_callback1;
int cnt_vel_set1;
int cnt_balance;
int cnt_rate;
float rate;
float last_rate=0;

float Angle_Velocity_Last_Bias = 0;
float Angle_Velocity_Integral = 0;
float X_balance_error = 0;
float Velocity_encoder_bias_integral = 0;

static float Smooth_Steer = 0.0f; 
static float Smooth_Speed = 0.0f; 

static int Last_Written_PWM = -1; 

// ==========================================
// 动态零点平稳校准算法 (双梯次极致逼近版)
// ==========================================
void Auto_Calibrate_Zero(void) 
{
    // 只有在车身处于近似平衡状态下才允许校准
    if (my_fabs(imu.rol) < 4.0f && param.run_flag == 1) 
    {
        static int calib_cnt = 0;
        static float speed_sum = 0.0f;
        static float speed_max = -10000.0f;
        static float speed_min = 10000.0f;
        
        float current_speed = odrive.now_speed0; // 获取当前飞轮速度
        
        // 数据收集
        speed_sum += current_speed; 
        if(current_speed > speed_max) speed_max = current_speed;
        if(current_speed < speed_min) speed_min = current_speed;
        
        calib_cnt++;
        
        // 收集 50 次数据 (在 25Hz 下约等于 2 秒钟的时间窗口)
        if (calib_cnt >= 50) 
        {
            // 1. 计算这 2 秒内的平均速度
            float avg_speed = speed_sum / 50.0f;
            
            // 2. 评估数据的变化情况 (抖动幅度)
            float speed_diff = speed_max - speed_min;
            
            // 综合判断：如果抖动幅度在正常平衡范围内（比如上下跳动没超过 150 RPM）
            if (speed_diff < 150.0f) 
            {
                // ==========================================================
                // 【极致双梯次校准】
                // 第一梯队：偏载明显（>3.0），用 0.005 快速拉回
                // ==========================================================
                if (avg_speed > 3.0f) {
                    Roll_Zero -= 0.005f; 
                } 
                else if (avg_speed < -3.0f) {
                    Roll_Zero += 0.005f;
                }
                // ==========================================================
                // 第二梯队：偏载微弱（0.5~3.0 之间，即出现 95% 和 5% 比例失调）
                // 用 0.001 极慢速雕刻榨干最后一点偏载，直到达到完美对称
                // ==========================================================
                else if (avg_speed > 0.5f) {
                    Roll_Zero -= 0.001f;
                }
                else if (avg_speed < -0.5f) {
                    Roll_Zero += 0.001f;
                }
            }

            // 限制最大校准范围，防止物理零点漂移过大导致危险
            if (Roll_Zero > 3.0f) Roll_Zero = 3.0f;
            if (Roll_Zero < -3.0f) Roll_Zero = -3.0f;
            
            // 重新清零，开始下一轮 2 秒的数据收集评估
            speed_sum = 0.0f;
            speed_max = -10000.0f;
            speed_min = 10000.0f;
            calib_cnt = 0;
        }
    }
    else
    {
        // 发生大角度倾斜时，数据不具备参考价值，保持原样即可
    }
}

// 定时器 2ms
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == &htim3)
    {
        imu_get(); 
        
        cnt_vel_set1++;
        cnt_balance++;   
        cnt_rate++;
        
        // 50Hz
        if(cnt_rate >= 10) 
        {
            cnt_rate = 0;
            rate_set(); 
        }
        
        if(param.scope_flag == 1) { balance(); cnt_balance=0; }
        
        // 2ms
        if(cnt_vel_set1 >= 1) {                  
             cnt_vel_callback1++;
             
             // 动量轮控制
             odrive_speed_ctrl(0, odrive.set_speed0); 
             odrive_vel_callback(0); // 获取动量轮速度
             
             cnt_vel_set1 = 0;
             
             // 10ms
             if(cnt_vel_callback1 >= 5) {
                 cnt_vel_callback1 = 0;
                 
                 // 后轮控制
                 odrive_speed_ctrl(1, -odrive.set_speed1); 
                 odrive_vel_callback(1); 
             }
        }
    }
}

// 核心控制逻辑
void rate_set()
{
    param.run_flag = 1; 

    if(param.run_flag == 1) 
    {
        // --- 1. 速度控制 (保持之前的柔性逻辑) ---
        float target_speed = 0.0f;
        if(my_fabs(Remote_Speed) >= 0.1f) target_speed = Remote_Speed;

        if(my_fabs(target_speed) < 0.01f && my_fabs(Smooth_Speed) > 0.01f) {
            Smooth_Speed += (0 - Smooth_Speed) * 0.05f; 
            if(my_fabs(Smooth_Speed) < 0.05f) Smooth_Speed = 0.0f;
        } else {
            Smooth_Speed += (target_speed - Smooth_Speed) * 0.02f; 
        }
        odrive.set_speed1 = Smooth_Speed;
        
        
        // ============================================================
        // --- 2. 转向控制 (匀速 + 1550 回中补偿) ---
        // ============================================================
        
        float final_target_steer = Remote_Steer;

        if (my_fabs(Remote_Steer) < 0.001f)
        {
            if (Smooth_Steer > 0.001f) 
            {
                final_target_steer = -0.1f; 
            }
            else
            {
                final_target_steer = 0.0f;
            }
        }

        // ============================================================
        // 【核心算法】线性匀速逼近 (Linear Ramp)
        // ============================================================
        float steer_diff = final_target_steer - Smooth_Steer;

        if(my_fabs(steer_diff) < STEP_SIZE) 
        {
            Smooth_Steer = final_target_steer;
        }
        else
        {
            if (steer_diff > 0)
                Smooth_Steer += STEP_SIZE;
            else
                Smooth_Steer -= STEP_SIZE;
        }
        
        // --- 计算目标 PWM ---
        float steer_pwm_delta = -Smooth_Steer * 600.0f; 
        int target_pwm = SERVO_CENTER_PWM + (int)steer_pwm_delta;

        // 硬件限位
        if(target_pwm > SERVO_MAX_PWM) target_pwm = SERVO_MAX_PWM;
        if(target_pwm < SERVO_MIN_PWM) target_pwm = SERVO_MIN_PWM;

        // --- 静默锁定 ---
        if (target_pwm != Last_Written_PWM)
        {
            servo_set_duty(target_pwm);
            Last_Written_PWM = target_pwm; 
        }
        
        // 动态零点补偿合成 (基础零点 + 舵机偏转补偿 + 自动寻找的绝对零点补偿)
        float pwm_delta = (float)(target_pwm - SERVO_CENTER_PWM);
        param.angular_zero = BASE_ANGLE_ZERO + (pwm_delta * COMPENSATE_K) + Roll_Zero;
    }
    else
    {
        odrive.set_speed1 = 0;
        servo_set_duty(SERVO_CENTER_PWM);
        Last_Written_PWM = SERVO_CENTER_PWM; 
        Smooth_Steer = 0.0f; 
        Smooth_Speed = 0.0f;
    }
}

// ---------------- PID ----------------
void param_init(){
    param.angular_kp = -5.0f;        
    param.angular_ki = 0;
    param.angular_kd = -1.5f;        
    param.angular_v_kp = -20.0f;     
    param.angular_v_ki = -0.001; 
    param.angular_v_kd =-2.0f; 
    param.fly_wheel_speed_kp = 0.04; 
    param.fly_wheel_speed_ki = 0;  
    param.fly_wheel_speed_kd = 0;
    param.zero_speed_kp=0;
    param.zero_speed_kd=0;
    param.zero_speed_ki=0;
    param.angular_zero = BASE_ANGLE_ZERO + Roll_Zero; 
    param.scope_flag = 1;
    param.run_flag=1;
    param.Steer_Kp = 1.5;
    param.Steer_Ki = 0.2;
    param.Steer_Kd = 0;
    Angle_Velocity_Integral = 0;
    Angle_Velocity_Last_Bias = 0;
    X_balance_error = 0;
    Velocity_encoder_bias_integral = 0;
}

float Angle_Velocity(float Gyro,float Gyro_Target) {
    float Angle_Velocity_Bias;
    float PWM_Out;
    Angle_Velocity_Bias = Gyro_Target - Gyro; 
    Angle_Velocity_Integral+=Angle_Velocity_Bias;
    if(Angle_Velocity_Integral > 10000) Angle_Velocity_Integral =10000;                          
    if(Angle_Velocity_Integral < -10000) Angle_Velocity_Integral = -10000;                  
    PWM_Out = param.angular_v_kp * Angle_Velocity_Bias + param.angular_v_ki * Angle_Velocity_Integral + param.angular_v_kd * (Angle_Velocity_Bias - Angle_Velocity_Last_Bias);
    Angle_Velocity_Last_Bias = Angle_Velocity_Bias;
    return PWM_Out;
}

float X_balance_Control(float Angle,float Angle_Zero,float gyro) {
     float PWM,Bias;
     Bias=Angle-Angle_Zero;                               
     X_balance_error+=Bias;                                   
     if(X_balance_error>+30) X_balance_error=+30;                         
     if(X_balance_error<-30) X_balance_error=-30;                         
     PWM=param.angular_kp*Bias + param.angular_ki*X_balance_error + (gyro)*param.angular_kd;        
     return PWM;
}

float Velocity_Control(float encoder,float target_encoder) {
    float encoder_bias,Velocity;
    encoder_bias = encoder - target_encoder;
    Velocity_encoder_bias_integral += encoder_bias;
    if(Velocity_encoder_bias_integral > +200) Velocity_encoder_bias_integral = +200;                          
    if(Velocity_encoder_bias_integral < -200) Velocity_encoder_bias_integral = -200;                          
    Velocity = encoder_bias * param.fly_wheel_speed_kp + Velocity_encoder_bias_integral * param.fly_wheel_speed_ki/1000;
    return Velocity;
}

void balance(void) {
    // 【倒地保护逻辑】(8度)
    if (my_fabs(imu.rol) > PROTECT_ANGLE) 
    {
        odrive.set_speed0 = 0; // 动量轮停
        odrive.set_speed1 = 0; // 后轮停 (如果需要)
        PWM_accel = 0;         // 清空速度积分
        X_balance_error = 0;   // 清空平衡积分
        Angle_Velocity_Integral = 0; // 清空速度积分
        Velocity_encoder_bias_integral = 0; // 清空动量轮速度积分
        return; 
    }

    // 2. 正常的平衡控制逻辑
    cnt1++;
    if(cnt1 >= 20) {
        float raw_speed_output = Velocity_Control(odrive.now_speed0, 0);
        raw_speed_output = -raw_speed_output; 
        PWM_accel = PWM_accel * 0.9f + raw_speed_output * 0.1f;
        cnt1 = 0;
        if(PWM_accel > 1.5f) PWM_accel = 1.5f;
        if(PWM_accel < -1.5f) PWM_accel = -1.5f;
        
        // 在 25Hz 的循环里调用极致的双梯次均值滤波校准
        Auto_Calibrate_Zero();
    }
    
    PWM_X = X_balance_Control(imu.rol, param.angular_zero + PWM_accel, imu.vx);              
    PWM_Final = Angle_Velocity(imu.vx, PWM_X);                                                 
    odrive.set_speed0 = PWM_Final;                              
}

int my_abs(int x) { if(x>=0) return x; else return -x; }
float my_fabs(float x) { if(x>=0) return x; else return -x; }
