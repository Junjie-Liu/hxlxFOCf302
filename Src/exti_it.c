
#include "stm32f3xx_hal.h"
#include "board.h"
#include "QEI.h"

/* ������ESC�йص��жϱ��뱣����ͬ�����ȼ���ֹ�໥Ƕ�� */
	
/* ����io�����жϴ��� */
void EXTI2_IRQHandler(void)
{
	/* LAN_ESC_SYNC0_Interrupt() */
  /* EXTI line interrupt detected */
//  if(__HAL_GPIO_EXTI_GET_FLAG(LAN_ESC_SYNC0_PIN) != RESET)
//  {
//    __HAL_GPIO_EXTI_CLEAR_FLAG(LAN_ESC_SYNC0_PIN);
//	}
}

void EXTI9_5_IRQHandler(void)
{
	/* LAN_ESC_SYNC1_Interrupt() */
  /* EXTI line interrupt detected */
//  if(__HAL_GPIO_EXTI_GET_FLAG(LAN_ESC_SYNC1_PIN) != RESET)
//  {
//    __HAL_GPIO_EXTI_CLEAR_FLAG(LAN_ESC_SYNC1_PIN);
//  }		
}

void EXTI15_10_IRQHandler(void)
{
	QEI_CHI_Interrupt();
	
	/* LAN_ESC_IRQ_Interrupt() */
  /* EXTI line interrupt detected */
//  if(__HAL_GPIO_EXTI_GET_FLAG(LAN_ESC_IRQ_PIN) != RESET)
//  {
//    __HAL_GPIO_EXTI_CLEAR_FLAG(LAN_ESC_IRQ_PIN);
//  }	
}