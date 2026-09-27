/**
  ******************************************************************************
  * @file    key.c
  * @brief   按键扫描（KEY0，接在 GPIOE4，上拉输入）
  ******************************************************************************
  * 【用法】Key_Scan() 返回 '1' 表示按下，否则返回 0。
  *
  * 【实现要点】
  *   - key_up 状态位：保证"一次按下只返回一次 '1'"，不是长按连发
  *   - HAL_Delay(20)：简单粗暴的软件消抖（按键是低频功能，阻塞可接受）
  *
  * ⚠️ 因为是【阻塞式】消抖，不要在中断里调用 Key_Scan()。
  ******************************************************************************
  */
#include "key.h"

void key_init(void) {
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

uint8_t Key_Scan(void)
{
	static uint8_t key_up=1;
	
	if(key_up && KEY0==0)
		{
			key_up=0;
			HAL_Delay(20);
			if(KEY0==0) return '1';
		}
	if(!(KEY0==0)) key_up=1;
	return 0;
}
