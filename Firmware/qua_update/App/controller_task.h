#ifndef _CONTROLLER_TASK_
#define _CONTROLLER_TASK_

#include "stm32f4xx_hal.h"

#define CONTROLLER_PERIOD 5
typedef struct
{
  int16_t         motor_pwn[4];
	
	int16_t         roll_ref;
	int16_t         pitch_ref;
	int16_t         yaw_ref;
	
  int16_t         angle_roll_fdb;
	int16_t         angle_roll_ref;
	int16_t         rate_roll_fdb;
	int16_t         rate_roll_ref;
	
  int16_t         angle_pitch_fdb;
	int16_t         angle_pitch_ref;
	int16_t         rate_pitch_fdb;
	int16_t         rate_pitch_ref;	
	
  int16_t         angle_yaw_fdb;
	int16_t         angle_yaw_ref;
	int16_t         rate_yaw_fdb;
	int16_t         rate_yaw_ref;	
	

} controller_t;

extern controller_t controller;

void controller_task(void const *argu);
void controller_init(void);
#endif
