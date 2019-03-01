#include "wireless_task.h"
#include "bsp_nrf24.h"
#include "cmsis_os.h"
uint8_t receieve_buf[16];
remoter_data_t remoter_data;

void wireless_task(void const *argument)
{
  for(;;)
  {
		if(NRF24L01_RxPacket(receieve_buf)== 0)
		{
			wireless_switch();
		}	
		osDelay(5);
  }

}

void wireless_switch()
{
  remoter_data.SW_L = receieve_buf[1];
  remoter_data.SW_R = receieve_buf[2];	
	remoter_data.thrus_value = receieve_buf[3] << 8| receieve_buf[4];
}
