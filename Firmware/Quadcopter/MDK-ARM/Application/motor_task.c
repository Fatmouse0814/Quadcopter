#include "motor_task.h"
#include "cmsis_os.h"
#include "tim.h"
#include "nrf24.h"
#include "stdlib.h"
#include "string.h"
#include "imu_task.h"
#include "pid.h"
uint32_t motor_time_last;
motor_type motor;
int motor_time_ms;
int thrus_speed;
int thrus_judge = 0;
pid_t pid_yaw             = {0};
pid_t pid_pitch           = {0};
pid_t pid_roll            = {0};
pid_t pid_yaw_spd         = {0};
pid_t pid_yaw_angle       = {0};
pid_t pid_pitch_spd       = {0};
pid_t pid_pitch_angle     = {0};
pid_t pid_roll_spd        = {0};
pid_t pid_roll_angle      = {0};
void MotorTask(void const * argument)
{
	uint32_t motor_wake_time = osKernelSysTick();
  for(;;)
  {
		    motor_time_ms = HAL_GetTick() - motor_time_last;
    motor_time_last = HAL_GetTick();
		if(buf_rx[0] == 0)
		{
			thrus_judge += 1;
			if(thrus_judge < 20)
			{
				buf_rx[0] = 's';
			}
			else
			{
			}
		}
		else
		{
			thrus_judge = 0;
		}
		if(buf_rx[0] == 's')
		{
			motor.pitch_angle_ref = imu.imu_data.pitch;
			motor.pitch_angle_fdb = 0;
			motor.pitch_spd_ref =  imu.imu_data.gyro_x;
			
			motor.roll_angle_ref = imu.imu_data.roll;
			motor.roll_angle_fdb = 0;
			motor.roll_spd_ref = imu.imu_data.gyro_y;
			
			motor.yaw_angle_ref = imu.imu_data.yaw;
			motor.yaw_angle_fdb = 180;
			motor.yaw_spd_ref = imu.imu_data.gyro_z;
			pid_calc(&pid_pitch_angle, motor.pitch_angle_ref, motor.pitch_angle_fdb);
			pid_calc(&pid_pitch_spd , motor.pitch_spd_ref, pid_pitch_angle.out);
			pid_calc(&pid_roll_angle, motor.roll_angle_ref, motor.roll_angle_fdb);
			pid_calc(&pid_roll_spd , motor.roll_spd_ref, pid_roll_angle.out);
			pid_calc(&pid_yaw_angle, motor.yaw_angle_ref, motor.yaw_angle_fdb);
			pid_calc(&pid_yaw_spd , motor.yaw_spd_ref, pid_yaw_angle.out);
			motor.vpitch = pid_pitch_spd.out;
			motor.vroll = pid_roll_spd.out;
			thrus_speed = buf_rx[4] - 48;
			motor.vthrus = 1400 + thrus_speed * 10;
			motor.vyaw = pid_yaw_spd.out;
			
			motor_calc(motor.vpitch , motor.vroll , motor.vyaw , motor.vthrus , motor.motor_spd);
//			motor.motor_spd[0] = motor.vthrus;//motor1	FR
//			motor.motor_spd[1] = motor.vthrus;//motor2 BL	 
//			motor.motor_spd[2] = motor.vthrus;//motor3 FL
//			motor.motor_spd[3] = motor.vthrus;//motor4 BR
			motor_drive(motor.motor_spd);
			buf_rx[0] = 0;
		}
		else if(buf_rx[0] == 'p')
		{
			TIM1 -> CCR1 = 1000;
			TIM1 -> CCR2 = 1000;
			TIM1 -> CCR3 = 1000;
			TIM1 -> CCR4 = 1000;
		}
		else
		{
			TIM1 -> CCR1 = 0;
			TIM1 -> CCR2 = 0;
			TIM1 -> CCR3 = 0;
			TIM1 -> CCR4 = 0;
	//		motor_judge += 1;
		}
		
		osDelayUntil(&motor_wake_time, 5);
  }
}

void motor_init(void)
{
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	TIM1 -> CCR1 = 1000;
	TIM1 -> CCR2 = 1000;
	TIM1 -> CCR3 = 1000;
	TIM1 -> CCR4 = 1000;
	
		// pitch�ᴮ��PID����                     
  PID_struct_init(&pid_pitch_spd, POSITION_PID, 350, 50,
                  1.0f, 0.0f, 0.0f);
	PID_struct_init(&pid_pitch_angle, POSITION_PID, 350, 100,
                  20.0f, 0.0f, 0.0f);
	
	// roll�ᴮ��PID���� 								
  PID_struct_init(&pid_roll_spd, POSITION_PID, 350, 50,
                  1.0f, 0.0f, 0.0f);
	PID_struct_init(&pid_roll_angle, POSITION_PID, 350, 100,
									20.0f, 0.0f, 0.0f);
	
	// yaw�ᴮ��PID���� 								
  PID_struct_init(&pid_yaw_spd, POSITION_PID, 200, 50,
                  1.0f, 0.0f, 0.0f);
	PID_struct_init(&pid_yaw_angle, POSITION_PID, 200, 100,
									10.0f, 0.0f, 0.0f);
}

void motor_calc(float vpitch, float vroll, float vyaw, float vthrus, int16_t speed[])
{
	int16_t motor_rpm[4];
	 float   max = 0;
	float mix = 2000;
	  //find max item
	motor_rpm[0] = vthrus - vpitch + vroll - vyaw;//motor1	FR
	motor_rpm[1] = vthrus + vpitch - vroll - vyaw;//motor2 BL	 
	motor_rpm[2] = vthrus - vpitch - vroll + vyaw;//motor3 FL
	motor_rpm[3] = vthrus + vpitch + vroll + vyaw;//motor4 BR 
  for (uint8_t i = 0; i < 4; i++)
  {
    if (abs(motor_rpm[i]) > max)
      max = abs(motor_rpm[i]);
  }
//	for (uint8_t i = 0; i < 4; i++)
//  {
//    if (abs(motor_rpm[i]) < mix)
//      mix = abs(motor_rpm[i]);
//  }
  //equal proportion
  if (max > MAX_MOTOR_RPM)
  {
    float rate = MAX_MOTOR_RPM / max;
    for (uint8_t i = 0; i < 4; i++)
      motor_rpm[i] *= rate;
  }
//	if (mix < MIX_MOTOR_RPM)
//  {
//    float ratem = MIX_MOTOR_RPM / mix;
//    for (uint8_t i = 0; i < 4; i++)
//      motor_rpm[i] *= ratem;
//  }
  memcpy(speed, motor_rpm, 4*sizeof(int16_t));
}
void motor_drive(int16_t speed_pwn[])
{
	TIM1 -> CCR1 = speed_pwn[0];
	TIM1 -> CCR2 = speed_pwn[1];
	TIM1 -> CCR3 = speed_pwn[2];
	TIM1 -> CCR4 = speed_pwn[3];

}
	
	
	
	
