#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "cmsis_os.h"

typedef struct
{
//	float value;
//	float inital;
	float change_scale;
	float real_target;
	float limit_target;
	TickType_t ticks;
	TickType_t last_ticks;
//	TicksTypedef *SlopeTick;
}Slope_Struct;

void Slope_On(Slope_Struct *V);

#endif
