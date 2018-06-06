
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

#define PWM_IN_MOD  0xFFFF        /* PWM���� */

#define VEL_CLK_ENABLE __HAL_RCC_TIM3_CLK_ENABLE()
#define VEL_TIM  TIM3
#define VEL_CNT  TIM3->CNT
#define VEL_Interrupt TIM3_IRQHandler 
#define VEL_IRQn      TIM3_IRQn  

#define GEAR_X        101                            /* �����䴫���� */
#define QEI_X         4                              /* MCU QEI�趨�ı�Ƶ QEI 4x */
#define QEI_CPR  (1200*4)                            /* ���̵�ʵ�������Ͷ�ͷ�ı�Ƶ Optical CPR*ReaderX */
#define QEI_POLAR_COUNT (QEI_CPR*QEI_X/MT_POLAR)     /* ���ÿ�Լ��ļ���ֵ count per motor POLAR pair */       
#define QEI_MOD  (QEI_CPR*QEI_X*GEAR_X*4)            /* QEI�趨������ֵ CPR*QEIx*GEARx = 360�ȵļ���ֵ*/
#define QEI_MAX_POS  (QEI_MOD/2)                     /* ��������λ�� */
#define QEI_CNT_TO_THETA (32768*4096/QEI_POLAR_COUNT)>>12   /* QEI����ֵת��Ϊת�ӽǶȵ�ϵ�� */
#define QEI_VEL_FRE   1000                          /* QEIת�ٿ���Ƶ�� @Hz*/
#define QEI_VEL_GAPS  (PWM_FRE/QEI_VEL_FRE)          /* QEIת�ٿ��Ƽ��ֵ gaps to get QEI vel */
#define IMPOSSIBLE_QEI_VEL (QEI_POLAR_COUNT)       /* �����ܵ�ת��ֵ�������ж�ת�ٲ���ֵ�Ƿ���� */

#define QEI_PER100RPM_COUNT ((float)QEI_CPR*QEI_X*100/60/QEI_VEL_FRE) /* ÿ10rpmʱ��QEIת�ٲ���ֵ per 100RPM QEI count */

#define PWM_IN_CPR  (46410)                            /* ���̵�ʵ�������Ͷ�ͷ�ı�Ƶ Optical CPR*ReaderX */
#define PWM_IN_POLAR_COUNT (PWM_IN_CPR/MT_POLAR)     /* ���ÿ�Լ��ļ���ֵ count per motor POLAR pair */       

#define PWM_IN_MAX_POS  (PWM_IN_MOD)                     /* ��������λ�� */
#define PWM_IN_CNT_TO_THETA (32768*4096/PWM_IN_POLAR_COUNT)>>12   /* QEI����ֵת��Ϊת�ӽǶȵ�ϵ�� */
#define PWM_IN_VEL_FRE   1000                          /* QEIת�ٿ���Ƶ�� @Hz*/
#define PWM_IN_VEL_GAPS  (PWM_FRE/QEI_VEL_FRE)          /* QEIת�ٿ��Ƽ��ֵ gaps to get QEI vel */
#define PWM_IN_IMPOSSIBLE_QEI_VEL (PWM_IN_POLAR_COUNT)       /* �����ܵ�ת��ֵ�������ж�ת�ٲ���ֵ�Ƿ���� */

//#define MAG_IN_CPR  (65536)                            /* ���̵�ʵ�������Ͷ�ͷ�ı�Ƶ Optical CPR*ReaderX */
//#define MAG_IN_POLAR_COUNT (MAG_IN_CPR/MT_POLAR)     /* ���ÿ�Լ��ļ���ֵ count per motor POLAR pair */       
//#define MAG_IN_CNT_TO_THETA (32768*4096/MAG_IN_POLAR_COUNT)>>12   /* QEI����ֵת��Ϊת�ӽǶȵ�ϵ�� */
//#define MAG_IN_VEL_FRE   1000                          /* QEIת�ٿ���Ƶ�� @Hz*/
//#define MAG_IN_VEL_GAPS  (PWM_FRE/MAG_IN_VEL_FRE)          /* QEIת�ٿ��Ƽ��ֵ gaps to get QEI vel */
//#define MAG_IN_IMPOSSIBLE_QEI_VEL (MAG_IN_POLAR_COUNT)       /* �����ܵ�ת��ֵ�������ж�ת�ٲ���ֵ�Ƿ���� */

#define MAG_CPR  (65536)                             /* ���̵�ʵ������ */
#define MAG_POLAR_COUNT (MAG_CPR/MT_POLAR)           /* ���ÿ�Լ��ļ���ֵ count per motor POLAR pair */
#define MAG_HALF_ROUND_COUNT (MAG_CPR>>1)            /* �����Ȧ�ļ���ֵ */
#define MAG_MOD  (MAG_CPR)                           /* MAG�趨������ֵ = 360�ȵļ���ֵ*/
#define MAG_CNT_TO_THETA (32768*4096/MAG_POLAR_COUNT)>>12   /* QEI����ֵת��Ϊת�ӽǶȵ�ϵ�� */
#define MAG_THETA_TO_CNT (MAG_POLAR_COUNT)>>15       /* ת�ӽǶ�ת��ΪMAG����ֵ��ϵ�� */ 
#define MAG_VEL_FRE   1000                            /* MAGת�ٿ���Ƶ�� @Hz*/
#define MAG_VEL_GAPS  (PWM_FRE/MAG_VEL_FRE)          /* QEIת�ٿ��Ƽ��ֵ gaps to get MAG vel */
#define IMPOSSIBLE_MAG_VEL (MAG_POLAR_COUNT)         /* �����ܵ�ת��ֵ�������ж�ת�ٲ���ֵ�Ƿ���� */

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
