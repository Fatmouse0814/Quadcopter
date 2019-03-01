#include "mode_switch_task.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "controller_task.h"

extern osTimerId controller_timer_id;

void mode_switch_task(void const * argument)
{
	uint32_t motor_wake_time = osKernelSysTick();
	osTimerStart(controller_timer_id, CONTROLLER_PERIOD);
	
  for(;;)
	{
//		switch()
//		{
//		
//		}
	  osDelayUntil(&motor_wake_time, 5);
	}
}
