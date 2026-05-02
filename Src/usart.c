/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  * of the USART instances.
  ******************************************************************************
  * ... (版权声明保持不变) ...
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN 0 */
/*Snow  NVIC_SetPriority(UART8_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 1));*/
/* USER CODE END 0 */


/* UART7 init function */
void MX_UART7_Init(void)
{
  LL_USART_InitTypeDef USART_InitStruct;
  LL_GPIO_InitTypeDef GPIO_InitStruct;

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART7);

  /**UART7 GPIO Configuration
  PE8   ------> UART7_TX
  PE7   ------> UART7_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* UART7 interrupt Init */
  NVIC_SetPriority(UART7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),2, 2));
  NVIC_EnableIRQ(UART7_IRQn);

  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(UART7, &USART_InitStruct);

  LL_USART_ConfigAsyncMode(UART7);

  LL_USART_Enable(UART7);
}


/* UART8 init function */
void MX_UART8_Init(void)
{
  LL_USART_InitTypeDef USART_InitStruct;
  LL_GPIO_InitTypeDef GPIO_InitStruct;

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART8);

  /**UART8 GPIO Configuration
  PE1   ------> UART8_TX
  PE0   ------> UART8_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1|LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* UART8 interrupt Init */
  NVIC_SetPriority(UART8_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 1));
  NVIC_EnableIRQ(UART8_IRQn);

  USART_InitStruct.BaudRate = 460800;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(UART8, &USART_InitStruct);

  LL_USART_ConfigAsyncMode(UART8);

  LL_USART_Enable(UART8);
}

/* ================================================================= */
/* 【新增】USART6 初始化函数 (已修改为 A 板 DBUS 接口引脚)              */
/* 连接工控机 | 波特率 115200 | TX:PG14 RX:PG9                       */
/* ================================================================= */
void MX_USART6_UART_Init(void)
{
  LL_USART_InitTypeDef USART_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* 1. 开启时钟 */
  // 串口外设时钟
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART6);
  // 【关键修改】开启 GPIOG 时钟 (原来是 GPIOC)
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOG);

  /**USART6 GPIO Configuration
  PG14   ------> USART6_TX 
  PG9    ------> USART6_RX
  */
  // 【关键修改】引脚改为 Pin 14 和 Pin 9
  GPIO_InitStruct.Pin = LL_GPIO_PIN_14 | LL_GPIO_PIN_9;
  
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
  
  // 【关键修改】初始化 GPIOG (原来是 GPIOC)
  LL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* 2. 配置中断优先级 (设为 3，避免抢占其他重要中断) */
  NVIC_SetPriority(USART6_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0));
  NVIC_EnableIRQ(USART6_IRQn);

  /* 3. 配置串口参数 (115200) */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART6, &USART_InitStruct);

  LL_USART_ConfigAsyncMode(USART6);

  /* 4. 使能串口 */
  LL_USART_Enable(USART6);
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/