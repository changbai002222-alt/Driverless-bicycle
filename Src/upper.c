#include "upper.h"
#include "usart.h"
#define FHead 0xA5
int16_t delta_x_buf;//Í¼Ïñ·µ»ØÖµ
uint8_t buf[1];
uint8_t buf_temp[1];
uint8_t state;

uint8_t buf2[4];
uint8_t ii = 0;


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
			 if(buf[0] == 0x00)
			 {
			   state=2;
			 }
			 else if(buf[0] == 0x01)
			 {
				 in_flag=1;
			 }
			 else
				 state = 0;
			 break;
		 case 2:
			 buf_temp[0] = buf[0];
			state = 3;
			 break;
		 case 3:
			 delta_x_buf = buf_temp[0] << 8 | buf[0];
			state = 0;
			break;
		 default:
			 break;
	 }
	 LL_USART_EnableIT_RXNE(UART7);
 }
}
//int cnt_i;
//void upper_send(int steer,int mode)
//{
//	
//	
//	uint8_t buf[5];
//	if(cnt_i>=5)
//	{
//		buf[0]=0xa5;
//		if(mode==0)
//		{
//			buf[1]=0x00;
//			buf[2]=(steer>>8)&0xFF;
//			buf[3]=steer&0xFF;
//			buf[4]=buf[1]^buf[2]^buf[3];
//		}
//		else if(mode==1)
//		{
//			buf[1]=0x01;
//			buf[2]=0x01;
//			buf[3]=0x00;
//			buf[4]=buf[1]^buf[2]^buf[3];
//		}
//		cnt_i=0;
//	}
//	else if(cnt_i<5)
//	{
//		LL_USART_TransmitData8(UART7,buf[cnt_i]);
//		while((UART7->SR&0X40) == 0){};
//		cnt_i++;
//	}
//	
//}
void back_center_send(void)
{
	static uint8_t back_center_i = 0;
	LL_USART_TransmitData8(UART7,back_center_data[back_center_i]);
	while((UART7->SR&0X40) == 0){};
	back_center_i++;
	back_center_i %= 5;
}
