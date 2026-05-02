#include "servo.h"
#include "tim.h"
#include "stm32f4xx_hal.h"

// 兼容宏定义
#ifndef PWM_SetDuty
#define PWM_SetDuty(htim, channel, duty) __HAL_TIM_SET_COMPARE(htim, channel, duty)
#endif

void servo_init(void)
{
    // 1. 补全硬件引脚配置（你 gpio.c 漏掉的部分）
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;       // 复用推挽输出
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;    // 关联到 TIM2
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 2. 开启定时器 PWM 通道
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); 
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); 
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // 对应你插的 U 位置
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); 
    
    servo_set_duty(1500); // 初始中位
}

void servo_set_duty(int pulse)
{
    // 安全限幅
    if(pulse > 2500) pulse = 2500;
    if(pulse < 500)  pulse = 500;
    
    // 直接操作寄存器，确保最高优先级写入
    htim2.Instance->CCR1 = pulse;
    htim2.Instance->CCR2 = pulse;
    htim2.Instance->CCR3 = pulse; 
    htim2.Instance->CCR4 = pulse;
}