#include "motor_task.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "tim.h"
#include "nrf24.h"
uint32_t motor_time_last;
int motor_time_ms;
int thrus_speed;
void MotorTask(void const * argument)
{
	uint32_t motor_wake_time = osKernelSysTick();
  for(;;)
  {
		    motor_time_ms = HAL_GetTick() - motor_time_last;
    motor_time_last = HAL_GetTick();
		if(buf_rx[0] == 's')
		{
			thrus_speed = buf_rx[4]-48;
			thrus_speed = 1000 + 100*thrus_speed ;
			TIM1 -> CCR1 = thrus_speed;
			TIM1 -> CCR2 = thrus_speed;
			TIM1 -> CCR3 = thrus_speed;
			TIM1 -> CCR4 = thrus_speed;
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
}
