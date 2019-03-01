#include "wireless.h"
#include "nrf24.h"

uint8_t transmit_buf[16];
remoter_data_t remoter_data;
void wireless_transmit()
{

	  switch_status();
		transmit_buf[0] = 0xff;
	  transmit_buf[1] = remoter_data.SW_L;
	  transmit_buf[2] = remoter_data.SW_R;
	  transmit_buf[3] = remoter_data.thrus_value >> 8;
	  transmit_buf[4] = remoter_data.thrus_value;	
		if(NRF24L01_TxPacket(transmit_buf) == TX_OK)
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
		}
		else 
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
		}
}

void wireless_init()
{
	remoter_data.SW_L = SW_L_UP;
	remoter_data.SW_R = SW_R_UP;
	remoter_data.thrus_value = 0;
	remoter_data.yaw_value = 0;
  remoter_data.pitch_value = 0;
	remoter_data.roll_value = 0;
}

void switch_status()
{
		if(HAL_GPIO_ReadPin(GPIOB, SW_L_U_Pin) == 0)
		{
		  remoter_data.SW_L = SW_L_DOWN;
		}
		else if(HAL_GPIO_ReadPin(GPIOB, SW_L_D_Pin) == 0)
		{
		  remoter_data.SW_L = SW_L_UP;
		}
		else
		{
		  remoter_data.SW_L = SW_L_MIDDLE;		
		}
		if(HAL_GPIO_ReadPin(GPIOC, SW_R_U_Pin) == 0)
		{
		  remoter_data.SW_R = SW_R_DOWN;
		}
		else if(HAL_GPIO_ReadPin(GPIOC, SW_R_D_Pin) == 0)
		{
		  remoter_data.SW_R = SW_R_UP;
		}
		else
		{
			remoter_data.SW_R = SW_R_MIDDLE;
		}


}
