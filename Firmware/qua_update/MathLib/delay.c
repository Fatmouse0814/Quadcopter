#include "delay.h"

void delay_us(uint32_t nus)
{		
	while(nus--)
	{
		for(int n=0;n<42;n++);
	}
}

//??nms
//nms:????ms?
void delay_ms(uint16_t nms)
{
	HAL_Delay(nms);
}

