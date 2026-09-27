/**
  ******************************************************************************
  * @file    upper.c
  * @brief   上位机 / 视觉模块通信（UART7）
  ******************************************************************************
  * 【用途】与"上位机 / 视觉模块"通信：
  *     接收 —— 图像处理结果（delta_x_buf / error_y：目标相对画面中心的偏差）
  *     发送 —— 回中指令 back_center_data
  *
  * 【帧格式】0xA5 帧头 + 异或校验
  *   back_center_data = {0xA5, 0x01, 0x01, 0x00, 0x01^0x01^0x00}
  *                       头    类型   数据1  数据2  校验(前面几字节异或)
  *
  * ⚠️【已知未完成】UART7_IRQHandler 目前是空实现 —— 视觉数据的接收还没写。
  ******************************************************************************
  */
#include "upper.h"
#include "usart.h"
#include "task.h"
#include "imu.h"
extern imu_t imu;
#define FHead 0xA5
int16_t delta_x_buf;//图像返回值
int16_t error_y;
uint8_t buf[1];
uint8_t buf_temp[1];
uint8_t state;
int cnttt;
uint8_t buf2[4];
uint8_t ii = 0;
uint8_t low_speed_flag=0;
uint8_t head_buf=0;//记录帧头
uint8_t back_center_data[5] = {0xa5,0x01,0x01,0x00,0x01^0x01^0x00};
extern float distance;
void UART7_IRQHandler(void)
{
}

void back_center_send(void)
{
	static uint8_t back_center_i = 0;
	LL_USART_TransmitData8(UART7,back_center_data[back_center_i]);
	while((UART7->SR&0X40) == 0){};
	back_center_i++;
	back_center_i %= 5;
}
