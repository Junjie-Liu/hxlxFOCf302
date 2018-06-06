
#ifndef PWM_H
#define PWM_H

#include <stdint.h>

#define PWM_CLK_ENABLE __HAL_RCC_TIM1_CLK_ENABLE()
#define PWM_TIM  TIM1
#define PWM_BDTR TIM1->BDTR
#define PWM_CCER TIM1->CCER
#define PWM_CCR1 TIM1->CCR1
#define PWM_CCR2 TIM1->CCR2
#define PWM_CCR3 TIM1->CCR3
#define PWM_CCR4 TIM1->CCR4
#define PWM_Interrupt TIM1_CC_IRQHandler //TIM1_UP_TIM16_IRQHandler//
#define PWM_IRQn      TIM1_CC_IRQn //TIM1_UP_TIM16_IRQn//TIM1_CC_IRQn     
#define PWM_BREAK_Interrupt   TIM1_BRK_TIM15_IRQHandler 
#define PWM_BREAK_IRQn        TIM1_BRK_TIM15_IRQn      

#define PWM_FRE  14400       /* PWM频率@Hz */
#define PWM_MOD  2500         /* PWM周期*/
#define PWM_DEADTIME  0            /* PWM死区 */
#define PWM_TRIGGER_ADC_TIME (PWM_MOD-2) /* PWM触发ADC的时间 trigger adc time for ch4, relate to PWM_DEADTIME */

void pwm_init(void);
void pwm_start(void);
void update_pwm1_duty(uint16_t duty);
void update_pwm2_duty(uint16_t duty);
void update_pwm3_duty(uint16_t duty);
void update_pwm4_duty(uint16_t duty);	
void updata_pwm(void);
void stop_pwm(void);

#endif
