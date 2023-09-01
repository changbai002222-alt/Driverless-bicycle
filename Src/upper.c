#include "upper.h"
#include "usart.h"
#define FHead 0xA5
int8_t delta_x_buf;
uint8_t buf[1];
uint8_t state;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
 if(huart == &huart2)
 {
	//	HAL_UART_Transmit_IT(&huart2,(uint8_t*)buf,1);
	 switch(state)
	 {
		 case 0:
			 if(buf[0] == FHead)
			 {
				state = 1;
			 }
			 break;
		 case 1:
			 delta_x_buf = (int8_t)buf[0]-128;
			 state = 0;
			 break;
	 }
	 HAL_UART_Receive_IT(&huart2,(uint8_t*)buf,1);
 }
}

