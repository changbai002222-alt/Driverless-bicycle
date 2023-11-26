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
void UART7_IRQHandler(void)
{
 if(LL_USART_IsActiveFlag_RXNE(UART7) && LL_USART_IsEnabledIT_RXNE(UART7))
	{
		buf[0]=LL_USART_ReceiveData8(UART7);
		
	  buf2[ii++] = buf[0];
		if (ii == 4)
			ii=0;
	 switch(state)
	 {
		 case 0:
			 if(buf[0] == FHead)
			 {
				state = 1;
			 }
			 break;
		 case 1:
			 if(buf[0] == 0x00)//舵机打角
			 {
			   state=2;
				 
			 }
			 else if(buf[0] == 0x01)//积分完成
			 {
				 in_flag=1;
			 }
			 else if(buf[0]==0x02)//第一个黄线，减速
			 {
				 low_speed_flag=1;
			 }
			 else
				 state = 0;
			 head_buf = buf[0];
			 break;
		 case 2:
			 buf_temp[0] = buf[0];
			 state = 3;
			 break;
		 case 3:
			 if(head_buf==0x00)delta_x_buf = buf_temp[0] << 8 | buf[0];	 
			 state = 0;
			 break;
		 
		 default:
			 break;
	 }
	 LL_USART_EnableIT_RXNE(UART7);
 }
}

void back_center_send(void)
{
	static uint8_t back_center_i = 0;
	LL_USART_TransmitData8(UART7,back_center_data[back_center_i]);
	while((UART7->SR&0X40) == 0){};
	back_center_i++;
	back_center_i %= 5;
}
