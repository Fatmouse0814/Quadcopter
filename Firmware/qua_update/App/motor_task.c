#include "motor_task.h"
#include "tim.h"
#include "cmsis_os.h"
#include "usart.h"
#include "imu_task.h"
#include "wireless_task.h"
#include "controller_task.h"
uint8_t serial_send[25];
uint8_t sum;
void motor_task(void const * argument)
{
	osEvent event;
	for(;;)
  {
		event = osSignalWait(MOTOR_CONTROL, osWaitForever);
		
		if (event.status == osEventSignal)
    {
      if (event.value.signals & MOTOR_CONTROL)
      {
				switch(remoter_data.SW_L)
				{
					case SW_L_UP:
					{
						for(uint8_t i = 0; i< 4;i++)
						{
						controller.motor_pwn[i] = 1000;
						}
            motor_pwn_set(controller.motor_pwn);
					}
					break;
					case SW_L_MIDDLE:
					{
						for(uint8_t i = 0; i< 4;i++)
						{
						  controller.motor_pwn[i] = 1000;
						}
            motor_pwn_set(controller.motor_pwn);

					}
					break;
					case SW_L_DOWN:
					{
						remoter_data.thrus_value= 1100 + remoter_data.thrus_value*10;
						motor_calcu();
						motor_pwn_set(controller.motor_pwn);
					}
					break;
					default:
					{
						break;
					}
				}
				HAL_GPIO_TogglePin(GPIOD,LED_2_Pin);
				serial_send[0] = 0xAA;
				serial_send[1] = 0xAA;
				serial_send[2] = 0x06;
				serial_send[3] = 14;
				serial_send[4] = controller.angle_roll_fdb >> 8;
				serial_send[5] = controller.angle_roll_fdb;
				serial_send[6] = controller.angle_roll_ref >> 8;
				serial_send[7] = controller.angle_roll_ref;
				serial_send[8] = controller.roll_ref >> 8;
				serial_send[9] = controller.roll_ref;
				serial_send[10] = controller.rate_roll_fdb >> 8;
				serial_send[11] = controller.rate_roll_fdb;
				serial_send[12] = controller.rate_roll_ref >> 8;
				serial_send[13] = controller.rate_roll_ref;
				serial_send[14] = (int16_t)imu.imu_data.yaw >> 8;
				serial_send[15] = (int16_t)imu.imu_data.yaw;
				serial_send[16] = remoter_data.thrus_value >> 8;
				serial_send[17] = remoter_data.thrus_value;
//	serial_send[18] = 0x00
//	serial_send[19] = 0x00
//	serial_send[20] = 0x00;//03Z
//	serial_send[21] = 0x00;	
	sum = 0;
	for(uint8_t i=0;i<18;i++)
	{
		sum += serial_send[i];
	}
	serial_send[18]= sum ;
	
	HAL_UART_Transmit(&huart1, serial_send,19, 100);
				
			}
		}			
	}
}

void motor_init()
{
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	TIM1 -> CCR1 = 1000;
	TIM1 -> CCR2 = 1000;
	TIM1 -> CCR3 = 1000;
	TIM1 -> CCR4 = 1000;
}

void motor_pwn_set(int16_t *motor_pwn)
{
	TIM1 -> CCR1 = motor_pwn[0];
	TIM1 -> CCR2 = motor_pwn[1];
	TIM1 -> CCR3 = motor_pwn[2];
	TIM1 -> CCR4 = motor_pwn[3];
}

void motor_calcu()
{
	float   min = 2000;
	controller.roll_ref = controller.roll_ref /2.0f;
	controller.pitch_ref = controller.pitch_ref /2.0f;
  controller.motor_pwn[0] = remoter_data.thrus_value - controller.roll_ref ;
	controller.motor_pwn[1] = remoter_data.thrus_value - controller.roll_ref ;
	controller.motor_pwn[2] = remoter_data.thrus_value + controller.roll_ref ;
	controller.motor_pwn[3] = remoter_data.thrus_value + controller.roll_ref ;
//	  controller.motor_pwn[0] = remoter_data.thrus_value - controller.roll_ref + controller.pitch_ref;
//	controller.motor_pwn[1] = remoter_data.thrus_value - controller.roll_ref - controller.pitch_ref;
//	controller.motor_pwn[2] = remoter_data.thrus_value + controller.roll_ref + controller.pitch_ref;
//	controller.motor_pwn[3] = remoter_data.thrus_value + controller.roll_ref - controller.pitch_ref;
		//find max item
  for (uint8_t i = 0; i < 4; i++)
  {
    if (controller.motor_pwn[i] < min)
      min = controller.motor_pwn[i];
  }
  //equal proportion
  if (min < MIN_MOTOR_PWN)
  {
    float rate = MIN_MOTOR_PWN / min;
    for (uint8_t i = 0; i < 4; i++)
      controller.motor_pwn[i] *= rate;
  }
}
