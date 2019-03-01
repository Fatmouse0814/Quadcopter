#ifndef __IMU_TASK_H__
#define __IMU_TASK_H__

#define IMU_TASK_PERIOD 1

typedef  struct
{
	float pitch,roll,yaw;
	float gyro_x,gyro_y,gyro_z;
}MPU9250_Data;

typedef  struct __IMU_Data
{
	MPU9250_Data imu_data;
	float yaw_now;
	float	yaw_last;
	float yaw_angle_ref;
	float yaw_angle_fdb;
}IMUType;
extern IMUType imu;
void imu_param_init(void);
void imu_task(void const *argument);

#endif
