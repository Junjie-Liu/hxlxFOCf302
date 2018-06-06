
#include "stm32f3xx_hal.h"
#include "arm_math.h"
#include "board.h"
#include "main.h"
#include "pwm.h"
#include "fun.h"
#include "foc.h"
#include "QEI.h"

uint16_t Motor_Theta = 0;

uint8_t release_break(uint8_t servo_id)
{
  uint32_t i = 0;
  uint8_t dtheta = 10;     /* 角度增量 */
  uint16_t voltage = 1000; /* 起始电压 Max=32767 */
	uint16_t max_voltage = MAX_START_VOLTAGE;  /* 最大电压 Max=32767 */  
	
	/* CCW 旋转 */
	for( i = 0; i < 5000; i++)
	{
		HAL_Delay(1);
		
		/* 缓启动 */
		if(voltage < max_voltage)
			voltage += 16;

		test_motor(0, 0, voltage);
	}

//	while(Motor_Theta != 0) /* 等待转子位置到0 */
//	{
//		HAL_Delay(1);
//		test_motor(dtheta, -1, voltage);
//	}

	return 1;
}

void test_motor(uint8_t dtheta, int8_t dir, uint16_t voltage)
{
  Motor_Theta += dir*dtheta;
  Motor_Theta %= 0x7FFF;
  
  openloop_run_motor(Motor_Theta, voltage);
  updata_pwm();  
}
