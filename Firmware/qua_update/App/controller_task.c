#include "controller_task.h"
#include "cmsis_os.h"
#include "motor_task.h"
#include "pid.h"
#include "imu_task.h"

extern TaskHandle_t motor_t;
controller_t controller;

void controller_task(void const *argu)
{
	controller.angle_roll_fdb = imu.imu_data.roll;
	controller.angle_roll_ref = 0;
	pid_calc(&pid_angle_roll, controller.angle_roll_fdb, controller.angle_roll_ref);
	controller.rate_roll_ref = pid_angle_roll.out;
	controller.rate_roll_ref = 0;
	controller.rate_roll_fdb = imu.imu_data.gyro_x;
	pid_calc(&pid_rate_roll, controller.rate_roll_fdb, controller.rate_roll_ref);
	controller.roll_ref = pid_rate_roll.out;
	
	controller.angle_pitch_fdb = imu.imu_data.pitch;
//	controller.angle_pitch_ref = 0;
	pid_calc(&pid_angle_pitch, controller.angle_pitch_fdb, controller.angle_pitch_ref);
	controller.rate_pitch_ref = pid_angle_pitch.out;
	controller.rate_pitch_fdb = -1.0f*imu.imu_data.gyro_y;
	pid_calc(&pid_rate_pitch, controller.rate_pitch_fdb, controller.rate_pitch_ref);
	controller.pitch_ref = pid_rate_pitch.out;

//	controller.angle_yaw_fdb = imu.imu_data.yaw;
//	controller.angle_yaw_ref = 0;
//	pid_calc(&pid_angle_yaw, controller.angle_yaw_fdb, controller.angle_yaw_ref);
//	controller.rate_yaw_ref = pid_angle_yaw.out;
//	controller.rate_yaw_fdb = imu.imu_data.gyro_z;
//	pid_calc(&pid_rate_yaw, controller.rate_yaw_fdb, controller.rate_yaw_ref);
//	controller.yaw_ref = pid_rate_yaw.out;	
	osSignalSet(motor_t, MOTOR_CONTROL);
}

void controller_init()
{
	PID_struct_init(&pid_angle_roll, POSITION_PID, 500, 400,
									1.1f,0.0f,0.0f	);  
	PID_struct_init(&pid_angle_pitch, POSITION_PID, 500, 100,
									1.0f,0.0f,0.0f	);  
	PID_struct_init(&pid_angle_yaw, POSITION_PID, 500, 100,
									0.1f,	0.0f,0.0f	);  
	PID_struct_init(&pid_rate_roll, POSITION_PID, 100, 50,
									1.0f,0.08f,0.0f	);  
	PID_struct_init(&pid_rate_pitch, POSITION_PID, 100, 50,
									1.0f,0.0f,0.0f	);  
	PID_struct_init(&pid_rate_yaw, POSITION_PID, 500, 100,
									0.1f,	0.0f,0.0f	);  
}
