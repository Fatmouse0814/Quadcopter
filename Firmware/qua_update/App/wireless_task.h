#ifndef _WIRELESS_TASK_
#define _WIRELESS_TASK_

#include "stm32f4xx_hal.h"
extern uint8_t receieve_buf[16];
typedef enum 
{
  SW_L_UP = 0x00,
	SW_L_MIDDLE = 0x01,
	SW_L_DOWN = 0x02
} SW_L_VALUE;

typedef enum 
{
  SW_R_UP = 0x00,
	SW_R_MIDDLE = 0x01,
	SW_R_DOWN = 0x02
} SW_R_VALUE;

typedef  struct
{
	uint8_t SW_L;
	uint8_t SW_R;
	int16_t thrus_value;
	int16_t yaw_value;
	int16_t pitch_value;
	int16_t roll_value;
}remoter_data_t;

extern remoter_data_t remoter_data;

void wireless_task(void const *argument);
void wireless_switch(void);

#endif
