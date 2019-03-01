#ifndef _MOTOR_TASK_H_
#define _MOTOR_TASK_H_

#include "stm32f4xx_hal.h"

#define MIN_MOTOR_PWN   1200		//底盘电机转速限制极限
#define MOTOR_CONTROL ( 1 << 2 )

void motor_task(void const * argument);
void motor_init(void);
void motor_pwn_set(int16_t *motor_pwn);
void motor_calcu(void);
#endif
