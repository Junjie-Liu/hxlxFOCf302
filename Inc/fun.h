
#ifndef FUN_H
#define FUN_H

#include <stdint.h>	

#define MAX_START_VOLTAGE 10000   /* ����ʼ��������ѹ, 100%ĸ�ߵ�ѹ = 32768 */


uint8_t release_break(uint8_t servo_id);
void test_motor(uint8_t dtheta, int8_t dir, uint16_t voltage);
	
#endif

