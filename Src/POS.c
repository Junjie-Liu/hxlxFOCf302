
#include "stm32f3xx_hal.h"
#include "board.h"
#include "main.h"
#include "QEI.h"
#include "foc.h"

int32_t search_zero_potion(uint32_t pos)
{
	static int32_t vel_lpfk = 16;  
	static int32_t vel_state = 0;
	static int32_t vel_lpf = 0;
	int32_t vel = 0;
	int32_t dir = 0;
//	int32_t dpos = (int32_t)pos - AEAT9000_HALF_COUNT;
//	
//	if( dpos > 0 )
//	{
//		dir = 1;
//		vel = (AEAT9000_MAX_COUNT - pos);
//		
//		/* 速度限幅 */
//		//vel control @1-2KHz
//		if(vel > 1024) 
//			vel >>= 7;
//		else if( vel > 0) 
//			vel = 8;		
//	}
//	
//	if( dpos < 0 )
//	{
//	  dir = -1;
//		vel = ((int32_t)0 - pos);
//		
//		/* 速度限幅 */	
//		//vel control @1-2KHz		
//		if(vel < -1024) 
//			vel >>= 7;
//		else if( vel < 0) 
//			vel = -8;		
//	}

//	/* 速度滤波 */
//	vel_state += (vel - vel_lpf)*vel_lpfk;
//	vel_lpf = vel_state>>15;

//	/* 准确停止 */
//	if( pos == 0 /*|| aeat9000_pos == (AEAT9000_MAX_COUNT-1) */)
//	{
//		return 0;
//	}
	
	return vel_lpf;
}
