/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (集成数据上报与零点校准)
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "can.h"
#include "tim.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "odrive.h"
#include "imu.h"     // 【关键】必须包含，才能识别 imu 结构体
#include "servo.h"
#include "task.h"
#include "upper.h"
#include "oled.h"
#include "key.h"
#include <string.h> 
#include <stdio.h>   // 【关键】必须包含，用于 sprintf

// 【关键】引入 LL 库的 USART 头文件
#include "stm32f4xx_ll_usart.h" 
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int key_flag = 0;
int key_times = 0;

// ============================================================
// 全局控制变量 (由 stm32f4xx_it.c 中的串口中断修改)
// ============================================================
volatile float Remote_Speed = 0.0f; 
volatile float Remote_Steer = 0.0f;
volatile float Roll_Zero    = 0.0f; // 【新增】零点校准偏移值，初始为0

// ============================================================
// 【外部变量引用】用于获取真实传感器数据
// ============================================================
// 1. 引用 imu.c 定义的结构体 (里面包含 rol, pit, yaw 等)
extern imu_t imu; 

// 2. 引用 odrive.c 定义的电机速度 (我们在 odrive.c 里刚加的)
extern volatile float v_momentum;    
extern volatile float v_rear_wheel;  

// ============================================================
// 【调试变量】
// task.c 会自动检测：如果没有 Python 指令，就使用这个值控制舵机
// ============================================================
volatile int Debug_Servo_Pulse = 1500; 

// 数据发送缓冲区
char Tx_Buffer[128];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/**
  * @brief  使用 LL 库发送字符串 (阻塞式发送，效率高)
  */
void LL_UART_SendString(USART_TypeDef *USARTx, char *str)
{
    while (*str)
    {
        while (!LL_USART_IsActiveFlag_TXE(USARTx)) {}
        LL_USART_TransmitData8(USARTx, *str++);
    }
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  程序入口
  * @retval None
*/
int main(void)
{ 
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* 重置外设，初始化 Flash 和 Systick */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* 配置系统时钟 (168MHz) */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* 初始化已配置的外设 */
  MX_GPIO_Init();
  MX_CAN2_Init();
  MX_TIM3_Init();    // 2ms 控制中断定时器
  MX_TIM2_Init();    // 舵机 PWM 定时器
  MX_I2C2_Init();
  MX_UART8_Init();
  MX_UART7_Init();
  MX_USART6_UART_Init(); // 与电脑/工控机通信

  /* USER CODE BEGIN 2 */
    
    // 1. 系统组件初始化
    odrive_init();   // 初始化电机通信
    imu_init();      // 初始化陀螺仪解码
    param_init();    // 初始化 PID 参数
    
    // 2. 中断开启 (使用 LL 库开启，响应更快)
    LL_USART_EnableIT_RXNE(UART8);  // 陀螺仪接收
    LL_USART_EnableIT_RXNE(UART7);  // 遥控器接收 (如有)
    LL_USART_EnableIT_RXNE(USART6); // 工控机指令接收 (包含零点校准)
    
    // 3. 舵机硬件开启
    servo_init();
    
    // 4. 开启核心控制中断
    HAL_TIM_Base_Start_IT(&htim3);
    
    HAL_Delay(1000); 
    key_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
        
        // ============================================================
        // 数据上报逻辑 (每 50ms 发送一次)
        // ============================================================
        static uint32_t Last_Send = 0;
        
        // 使用非阻塞延时，不影响其他逻辑运行
        if (HAL_GetTick() - Last_Send > 50) 
        {
            Last_Send = HAL_GetTick();

            // ----------------------------------------------------
            // 组包数据，格式: R:Roll角度,Z:零点,M:动量轮RPM,W:后轮RPM
            // ----------------------------------------------------
            // 注意：
            // 1. imu.rol 是原始角度，减去 Roll_Zero 才是当前用于平衡的“净角度”
            // 2. Roll_Zero 是当前设置的校准值
            // 3. %.2f 保留两位小数, %.0f 只显示整数
            // ----------------------------------------------------
            
            sprintf(Tx_Buffer, "R:%.2f,Z:%.2f,M:%.0f,W:%.0f\r\n", 
                    imu.rol,   // 发送给工控机显示的实际姿态
                    param.angular_zero,             // 发送当前的校准值，确认是否修改成功
                    v_momentum,            // 动量轮速度
                    v_rear_wheel           // 后轮速度
                   );
            
            // 发送数据到工控机 (USART6)
            LL_UART_SendString(USART6, Tx_Buffer);
        }
        
  }
  /* USER CODE END 3 */
}

/**
  * @brief 系统时钟配置 (保持不变)
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

void _Error_Handler(char *file, int line)
{
  while(1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */