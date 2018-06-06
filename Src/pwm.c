
#include "stm32f3xx_hal.h"
#include "board.h"
#include "main.h"
#include "pwm.h"
#include "adc.h"
#include "foc.h"

static TIM_HandleTypeDef pwm_tim;

void pwm_init(void)
{
  static TIM_ClockConfigTypeDef sClockSourceConfig;
  static TIM_MasterConfigTypeDef sMasterConfig;
  static TIM_OC_InitTypeDef sConfigOC;
  static TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  
  PWM_CLK_ENABLE;
  
  pwm_tim.Instance               = PWM_TIM;
  pwm_tim.Init.Prescaler         = 0;
  pwm_tim.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1;
  pwm_tim.Init.Period            = PWM_MOD;
  pwm_tim.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  pwm_tim.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&pwm_tim) != HAL_OK)
  {
    Error_Handler(HAL_TIM_Base_Init_ERROR);
  }
  
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&pwm_tim, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler(HAL_TIM_ConfigClockSource_ERROR);
  }
  
  if (HAL_TIM_PWM_Init(&pwm_tim) != HAL_OK)
  {
    Error_Handler(HAL_TIM_PWM_Init_ERROR);
  }

  //sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;  
  //sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_OC4REF_RISINGFALLING;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;;//TIM_MASTERSLAVEMODE_ENABLE;//TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&pwm_tim, &sMasterConfig) != HAL_OK)
  {
    Error_Handler(HAL_TIMEx_MasterConfigSynchronization_ERROR);
  }
	

  
  sConfigOC.OCMode       = TIM_OCMODE_PWM1;
  sConfigOC.Pulse        = 0;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode   = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&pwm_tim, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler(HAL_TIM_PWM_ConfigChannel_ERROR);
  }
  if (HAL_TIM_PWM_ConfigChannel(&pwm_tim, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler(HAL_TIM_PWM_ConfigChannel_ERROR);
  }
  if (HAL_TIM_PWM_ConfigChannel(&pwm_tim, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler(HAL_TIM_PWM_ConfigChannel_ERROR);
  }
	
  sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime         = PWM_DEADTIME;
  sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;//TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&pwm_tim, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler(HAL_TIMEx_ConfigBreakDeadTime_ERROR);
  }
  
  /* Enable the Peripheral */
  __HAL_TIM_ENABLE(&pwm_tim);	
	
}

void pwm_start(void)
{
  /* channel 4 used as trigger signal for adc sample */
  update_pwm4_duty(PWM_TRIGGER_ADC_TIME);
  
  PWM_CCER = 0x111; /* PWM IO enable */	
  //PWM_CCER = 0x333; /* PWM IO enable */
  
  /* Enable the TIM Update interrupt */
  //__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
  /* Enable the TIM CC4 interrupt */
  __HAL_TIM_ENABLE_IT(&pwm_tim, TIM_IT_CC4);	

  
  /* Peripheral interrupt init */
  HAL_NVIC_SetPriority(PWM_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(PWM_IRQn);	
	
	/* BREAK 中断过流保护 */
	__HAL_TIM_ENABLE_IT(&pwm_tim, TIM_IT_BREAK);
	
  /* Peripheral interrupt init */
  HAL_NVIC_SetPriority(PWM_BREAK_IRQn, 0, 0); /* 过流保护，最高优先级 */
  HAL_NVIC_EnableIRQ(PWM_BREAK_IRQn);		
	
	/* enable MOE bit start PWM OUTPUT */
	PWM_BDTR |= 0x8000;
}

void update_pwm1_duty(uint16_t duty)
{
  PWM_CCR1 = duty % PWM_MOD;
}

void update_pwm2_duty(uint16_t duty)
{
  PWM_CCR2 = duty % PWM_MOD;
}

void update_pwm3_duty(uint16_t duty)
{
  PWM_CCR3 = duty % PWM_MOD;
}

void update_pwm4_duty(uint16_t duty)
{
  PWM_CCR4 = duty % PWM_MOD;
}

void updata_pwm(void)
{
	/* 禁止中断，防止更新不同步 */
	__disable_irq();
  PWM_CCR1 = SVpwm.Pwm1;
  PWM_CCR2 = SVpwm.Pwm2;
  PWM_CCR3 = SVpwm.Pwm3; 
	__enable_irq();
}

void stop_pwm(void)
{
	/* 禁止中断，防止更新不同步 */
	__disable_irq();
  PWM_CCR1 = 0;
  PWM_CCR2 = 0;
  PWM_CCR3 = 0;
	__enable_irq();
}

/* 
* pwm interrupt
*
*/
void PWM_Interrupt(void)
{
	if( __HAL_TIM_GET_FLAG(&pwm_tim, TIM_FLAG_CC4) )
  //if((pwm_tim.Instance->SR & TIM_FLAG_CC4) == TIM_FLAG_CC4)
  {		
    __HAL_TIM_CLEAR_FLAG(&pwm_tim, TIM_FLAG_CC4);  /* clear interrupt flag */
    adc_start_sample(); /* trigger adc sample */
  }	
//	if( __HAL_TIM_GET_FLAG(&pwm_tim, TIM_FLAG_UPDATE) )
//  //if((pwm_tim.Instance->SR & TIM_FLAG_CC4) == TIM_FLAG_CC4)
//  {		
//    __HAL_TIM_CLEAR_FLAG(&pwm_tim, TIM_FLAG_UPDATE);  /* clear interrupt flag */
//    adc_start_sample(); /* trigger adc sample */
//  }		
}

void PWM_BREAK_Interrupt(void)
{
	if( __HAL_TIM_GET_FLAG(&pwm_tim, TIM_FLAG_BREAK) )
  //if((pwm_tim.Instance->SR & TIM_FLAG_BREAK) == TIM_FLAG_BREAK)
  {		
    __HAL_TIM_CLEAR_FLAG(&pwm_tim, TIM_FLAG_BREAK);  /* clear interrupt flag */
  }	
}

