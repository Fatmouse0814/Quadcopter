#ifndef _MOTOR_TASK_H_
#define _MOTOR_TASK_H_
#include "stm32f4xx_hal.h"
#define MAX_MOTOR_RPM 2000
#define MIX_MOTOR_RPM 1000
typedef struct
{
	float vpitch;
	float vroll;
	float vyaw;
	float vthrus;
	
	int16_t         yaw_spd_fdb;
  int16_t         yaw_spd_ref;
	int16_t         yaw_angle_fdb;
  int16_t         yaw_angle_ref;
	
	int16_t					pitch_spd_fdb;
  int16_t         pitch_spd_ref;
	int16_t         pitch_angle_fdb;
  int16_t         pitch_angle_ref;

	int16_t					roll_spd_fdb;
  int16_t         roll_spd_ref;
	int16_t         roll_angle_fdb;
  int16_t         roll_angle_ref;	
	
	int16_t         motor_spd[4];
	
} motor_type;

extern motor_type motor;

void MotorTask(void const * argument);
void motor_init(void);
void motor_calc(float vpitch, float vroll, float vyaw, float vthrus, int16_t speed[]);
void motor_drive(int16_t speed_pwn[]);

#endif
