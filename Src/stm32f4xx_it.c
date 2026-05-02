/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

// 【关键】引入 LL 库头文件 (用于高效操作串口)
#include "stm32f4xx_ll_usart.h"

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart6; // 引用 UART6 句柄

// ============================================================
// 【全局变量引用】引入 main.c 中定义的控制变量
// ============================================================
extern volatile float Remote_Speed;   // 遥控速度
extern volatile float Remote_Steer;   // 遥控转向
extern volatile float Roll_Zero;      // 零点校准值

/******************************************************************************/
/* Cortex-M4 Processor Interruption and Exception Handlers         */
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  while (1)
  {
  }
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  while (1)
  {
  }
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  while (1)
  {
  }
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/******************************************************************************/

/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim3);
}

/**
* @brief This function handles CAN2 RX0 interrupts.
*/
void CAN2_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan2);
}

/**
* @brief This function handles TIM4 global interrupt.
*/
void TIM4_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim4);
}

/* USER CODE BEGIN 1 */

// 静态变量，用于记录上一次收到的指令
static uint8_t Last_Command = 0;

/**
  * @brief  USART6 global interrupt.
  * @note   负责接收所有上位机指令
  */
void USART6_IRQHandler(void)
{
    // 检查 RXNE 标志 (接收寄存器非空)
    if(LL_USART_IsActiveFlag_RXNE(USART6) && LL_USART_IsEnabledIT_RXNE(USART6))
    {
        uint8_t data = LL_USART_ReceiveData8(USART6);

        // ============================================================
        // 【核心修复】 智能去重逻辑
        // 1. 如果是 '+' 或 '-' (调零点)，允许重复执行 (不做去重)，
        //    这样你狂点按钮时，零点值就会不断累加/累减。
        // 2. 其他指令 (如 'F' 前进)，如果按住不放，只执行一次，防止刷屏。
        // ============================================================
        if (data != '+' && data != '-' && data == Last_Command) 
        {
             return; // 重复指令，直接忽略
        }
        
        Last_Command = data; // 更新上一条指令

        switch(data)
        {
            // --- 运动控制 ---
            case 'F': Remote_Speed = -0.7f; break; // 前进
            case 'B': Remote_Speed = 0.7f;  break; // 后退

            case 'L': Remote_Steer = -0.5f; break;   // 大左
            case 'R': Remote_Steer = 0.5f;  break;   // 大右
            case 'A': Remote_Steer = -0.167f; break; // 小左
            case 'D': Remote_Steer = 0.167f;  break; // 小右

            case 'S': // 【全停】速度、转向全归零
                Remote_Speed = 0.0f;
                Remote_Steer = 0.0f;
                break;

            case 'C': // 【仅回中】只归零转向，保持速度不变
                Remote_Steer = 0.0f;
                break;

            // --- 零点校准 (现在可以无限连点了) ---
            case '+': 
                Roll_Zero += 0.05f; 
                break;
            
            case '-': 
                Roll_Zero -= 0.05f; 
                break;
            
            default: break;
        }
    }
    
    // 清除错误标志 (防止 ORE 溢出错误导致中断卡死)
    if (LL_USART_IsActiveFlag_ORE(USART6))
    {
        LL_USART_ClearFlag_ORE(USART6);
    }
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/