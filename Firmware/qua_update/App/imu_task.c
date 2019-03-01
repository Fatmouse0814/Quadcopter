#include "imu_task.h"
#include "cmsis_os.h"
#include "bsp_9250.h"
IMUType imu;
uint32_t imu_time_last;
int imu_time_ms;

void imu_task(void const *argument)
{
	 for(;;)
  {
    mpu_mpl_get_data(&imu.imu_data.pitch,&imu.imu_data.roll,&imu.imu_data.yaw,
										&imu.imu_data.gyro_x,&imu.imu_data.gyro_y,&imu.imu_data.gyro_z);
		osDelay(5);
	}

}
	
void imu_param_init(void)
{
	while(mpu_dmp_init()!=0);   //Õ”¬›“«≥ı ºªØ
	HAL_Delay(2000);            
}
