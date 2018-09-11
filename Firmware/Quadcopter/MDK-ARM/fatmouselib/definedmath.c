#include "definedmath.h"

void Slope_On(Slope_Struct *V)
{
	V->last_ticks = V->ticks;
	V->ticks = osKernelSysTick();
	if(V->real_target !=V->limit_target)
	{
		if(V->real_target < V->limit_target)//加操作
		{
			V->real_target = V->real_target + V->change_scale* (V->ticks-V->last_ticks);
			//V->real_target = V->real_target + 0x20;
			if(V->real_target > V->limit_target)//限幅
			{
				V->real_target =  V->limit_target;
			}
		}	
		else if(V->real_target > V->limit_target)//减操作
		{
			V->real_target =  V->real_target - V->change_scale* (V->ticks-V->last_ticks);
			if(V->real_target < V->limit_target)//限幅
			{
				V->real_target =  (short)V->limit_target;
			}
		}
	}
}
