#include "motor_task.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "tim.h"
#include "nrf24.h"
uint32_t motor_time_last;
int motor_time_ms;
void MotorTask(void const * argument)
{
	uint32_t motor_wake_time = osKernelSysTick();
  for(;;)
  {
		    motor_time_ms = HAL_GetTick() - motor_time_last;
    motor_time_last = HAL_GetTick();
		if(buf_rx[1] == 'd')
		{
			TIM1 -> CCR1 = 1000;
			TIM1 -> CCR2 = 1000;
			TIM1 -> CCR3 = 1000;
			TIM1 -> CCR4 =1000;
		}
		else if(buf_rx[1] == 'm')
		{

						TIM1 -> CCR1 = 1100;
			TIM1 -> CCR2 = 1100;
			TIM1 -> CCR3 = 1100;
			TIM1 -> CCR4 = 1100;
		}
		else if(buf_rx[1] == 'u')
		{			
		TIM1 -> CCR1 = 1200;
		TIM1 -> CCR2 = 1200;
		TIM1 -> CCR3 = 1200;
		TIM1 -> CCR4 = 1200;
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
