#include "upper.h"
#include "usart.h"
#define FHead 0xA5
int16_t delta_x_buf;//Í¼Ïñ·µ»ØÖµ
uint8_t buf[1];
uint8_t buf_temp[1];
uint8_t state;

uint8_t buf2[4];
uint8_t ii = 0;
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
void upper_send_data(uint8_t *buf,int len)
{
	int i = 0;
	for(i=0;i<len;i++)
	{
		LL_USART_TransmitData8(UART7,buf[i]);
		while((UART7->SR&0X40) == 0){};
	}
	
}
void upper_send(int steer)
{
	uint8_t buf[4];
	buf[0]=0xa5;
	buf[1]=0x00;
	buf[2]=(steer>>8)&0xFF;
	buf[3]=steer&0xFF;
	upper_send_data(buf,4);
	
}
