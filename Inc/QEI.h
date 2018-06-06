
#ifndef QEI_H
#define QEI_H

#include "foc_param.h"
#include "foc.h"
#include "pwm.h"

#define QEI_CLK_ENABLE __HAL_RCC_TIM2_CLK_ENABLE()
#define QEI_TIM  TIM2
#define QEI_CNT  TIM2->CNT
#define QEI_Interrupt TIM2_IRQHandler 
#define QEI_IRQn      TIM2_IRQn

#define PWM_IN_CLK_ENABLE __HAL_RCC_TIM3_CLK_ENABLE()
#define PWM_IN_TIM  TIM3
#define PWM_IN_CCER TIM3->CCER
#define PWM_IN_CCR1 TIM3->CCR1
#define PWM_IN_CCR2 TIM3->CCR2
#define PWM_IN_CCR3 TIM3->CCR3
#define PWM_IN_CCR4 TIM3->CCR4
#define PWM_IN_Interrupt TIM3_IRQHandler 
#define PWM_IN_IRQn      TIM3_IRQn     

#define PWM_IN_MOD  0xFFFF        /* PWM周期 */

#define VEL_CLK_ENABLE __HAL_RCC_TIM3_CLK_ENABLE()
#define VEL_TIM  TIM3
#define VEL_CNT  TIM3->CNT
#define VEL_Interrupt TIM3_IRQHandler 
#define VEL_IRQn      TIM3_IRQn  

#define GEAR_X        101                            /* 变速箱传动比 */
#define QEI_X         4                              /* MCU QEI设定的倍频 QEI 4x */
#define QEI_CPR  (1200*4)                            /* 码盘的实际线数和读头的倍频 Optical CPR*ReaderX */
#define QEI_POLAR_COUNT (QEI_CPR*QEI_X/MT_POLAR)     /* 电机每对极的计数值 count per motor POLAR pair */       
#define QEI_MOD  (QEI_CPR*QEI_X*GEAR_X*4)            /* QEI设定的周期值 CPR*QEIx*GEARx = 360度的计数值*/
#define QEI_MAX_POS  (QEI_MOD/2)                     /* 允许的最大位置 */
#define QEI_CNT_TO_THETA (32768*4096/QEI_POLAR_COUNT)>>12   /* QEI计数值转换为转子角度的系数 */
#define QEI_VEL_FRE   1000                          /* QEI转速控制频率 @Hz*/
#define QEI_VEL_GAPS  (PWM_FRE/QEI_VEL_FRE)          /* QEI转速控制间隔值 gaps to get QEI vel */
#define IMPOSSIBLE_QEI_VEL (QEI_POLAR_COUNT)       /* 不可能的转速值，用于判断转速测量值是否合理 */

#define QEI_PER100RPM_COUNT ((float)QEI_CPR*QEI_X*100/60/QEI_VEL_FRE) /* 每10rpm时的QEI转速测量值 per 100RPM QEI count */

#define PWM_IN_CPR  (46410)                            /* 码盘的实际线数和读头的倍频 Optical CPR*ReaderX */
#define PWM_IN_POLAR_COUNT (PWM_IN_CPR/MT_POLAR)     /* 电机每对极的计数值 count per motor POLAR pair */       

#define PWM_IN_MAX_POS  (PWM_IN_MOD)                     /* 允许的最大位置 */
#define PWM_IN_CNT_TO_THETA (32768*4096/PWM_IN_POLAR_COUNT)>>12   /* QEI计数值转换为转子角度的系数 */
#define PWM_IN_VEL_FRE   1000                          /* QEI转速控制频率 @Hz*/
#define PWM_IN_VEL_GAPS  (PWM_FRE/QEI_VEL_FRE)          /* QEI转速控制间隔值 gaps to get QEI vel */
#define PWM_IN_IMPOSSIBLE_QEI_VEL (PWM_IN_POLAR_COUNT)       /* 不可能的转速值，用于判断转速测量值是否合理 */

//#define MAG_IN_CPR  (65536)                            /* 码盘的实际线数和读头的倍频 Optical CPR*ReaderX */
//#define MAG_IN_POLAR_COUNT (MAG_IN_CPR/MT_POLAR)     /* 电机每对极的计数值 count per motor POLAR pair */       
//#define MAG_IN_CNT_TO_THETA (32768*4096/MAG_IN_POLAR_COUNT)>>12   /* QEI计数值转换为转子角度的系数 */
//#define MAG_IN_VEL_FRE   1000                          /* QEI转速控制频率 @Hz*/
//#define MAG_IN_VEL_GAPS  (PWM_FRE/MAG_IN_VEL_FRE)          /* QEI转速控制间隔值 gaps to get QEI vel */
//#define MAG_IN_IMPOSSIBLE_QEI_VEL (MAG_IN_POLAR_COUNT)       /* 不可能的转速值，用于判断转速测量值是否合理 */

#define MAG_CPR  (65536)                             /* 码盘的实际线数 */
#define MAG_POLAR_COUNT (MAG_CPR/MT_POLAR)           /* 电机每对极的计数值 count per motor POLAR pair */
#define MAG_HALF_ROUND_COUNT (MAG_CPR>>1)            /* 电机半圈的计数值 */
#define MAG_MOD  (MAG_CPR)                           /* MAG设定的周期值 = 360度的计数值*/
#define MAG_CNT_TO_THETA (32768*4096/MAG_POLAR_COUNT)>>12   /* QEI计数值转换为转子角度的系数 */
#define MAG_THETA_TO_CNT (MAG_POLAR_COUNT)>>15       /* 转子角度转换为MAG计数值的系数 */ 
#define MAG_VEL_FRE   1000                            /* MAG转速控制频率 @Hz*/
#define MAG_VEL_GAPS  (PWM_FRE/MAG_VEL_FRE)          /* QEI转速控制间隔值 gaps to get MAG vel */
#define IMPOSSIBLE_MAG_VEL (MAG_POLAR_COUNT)         /* 不可能的转速值，用于判断转速测量值是否合理 */

extern int32_t QEIpos;

void qei_init(void);
void qei_count_set(uint32_t count);
uint32_t get_qei_count(void);
void get_qei_vel_pos(int32_t *qvel, int32_t *qpos);
void get_mag_vel_pos(FOC_t *pFOC);
void get_pwm_vel_pos(int32_t *qvel, int32_t *qpos);
void QEI_CHI_Interrupt(void);

int32_t get_pwm_in_duty(void);
#endif
