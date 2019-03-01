#ifndef _WIRELESS_H_
#define _WIRELESS_H_

#include "stm32f1xx_hal.h"

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
void wireless_transmit(void);
void switch_status(void);
void wireless_init(void);
#endif
