
#ifndef FOC_PARAM_H
#define FOC_PARAM_H
	
#include <stdint.h>	
//UNIT:
//current: mA
//time: S
//resistance: ohm
//inductance: H

#define ADC_PER_MA   (32768.0f/5000);      /* ADC��������ֵת����ʵ��mAֵ��ϵ�� adc represent how many mA */
                            /* ʵ�ʵ���ֵ=ADC����ֵ*ADC_PER_MA(��λmA) */

/*ϵͳ PWM���� */
#ifndef PWM_MOD
#define PWM_MOD        2500   /* PWM���� */
#endif

/* MotorMode */
#define STOP_MODE      0  /* ֹͣ */
#define TORQUE_MODE    1  /* Ť��ģʽ */
#define VELOCITY_MODE  2  /* �ٶ�ģʽ */
#define POSITION_MODE  3  /* λ��ģʽ */
	
/* ErrorMode */	
#define UNDER_VOLTAGE  0  /* Ƿѹ���� */
#define OVER_VOLTAGE   1  /* ��ѹ���� */
#define OVER_CURRENT   2  /* �������� */

//big motor
#define MT_POLAR          7//21//7//11 /* ��������� pairs */
#define MT_MAX_TORQUE     30000 /* ������ 16384 = 5000 mA */
#define MT_MAX_VELOCITY   800 //(@1KHz)  /* ������ת��1000RPM/60/1000 = ÿ���������ڵ�ת�� */
                                         /* ÿ���������ڵ�ת��*MAG��ÿתMOD = ÿ���������ڵ����ת�� */

#define IDQ_LPFK   4096    /*  */ 

/* ϵͳ���в��� */
#define START_MODE TORQUE_MODE          /* ���Ĭ��ģʽ */

/* ϵͳ���ܲ��� */
#define QEI_EN           1      /* QEI ʹ�� if QEI enabled */
#define QEI_VEL_LPFK   8192      /* ת���˲�ϵ�� = QEI_VEL_LPFK*2^15  */ 
#define QEI_POS_LPFK   32768    /* ת���˲�ϵ�� = QEI_VEL_LPFK*2^15  */ 
#define MAG_ACC_LPFK   1024     /* ת���˲�ϵ�� = MAG_ACC_LPFK*2^15  */ 
#define MAG_VEL_LPFK   4096     /* ת���˲�ϵ�� = MAG_VEL_LPFK*2^15  */ 
#define MAG_POS_LPFK   512     /* ת���˲�ϵ�� = MAG_VEL_LPFK*2^15  */

/* PID���� */
#define Q15(Float_Value)	\
        ((Float_Value < 0.0f) ? (int16_t)(32768 * (Float_Value) - 0.5f) \
        : (int16_t)(32767 * (Float_Value) + 0.5f))

/* PI controllers tuning values - */
//******** D Control Loop Coefficients *******
#define     A123_D_CURRCNTR_PTERM       (int32_t)200//2000//500//Q15(0.06f)
#define     A123_D_CURRCNTR_ITERM       (int32_t)25//100//100 //Q15(0.000f)
#define     A123_D_CURRCNTR_CTERM       (int32_t)0    //Q15(0.f)
#define     A123_D_CURRCNTR_OUTMAX      (int32_t)31000 /* ����PWM������Ӱ�� */
#define     A123_D_CURRCNTR_SUMMAX      (int32_t)4000000*16

//******** Q Control Loop Coefficients *******
#define     A123_Q_CURRCNTR_PTERM       (int32_t)200//2000//500//Q15(0.06f)
#define     A123_Q_CURRCNTR_ITERM       (int32_t)25//100//100 //Q15(0.000f)
#define     A123_Q_CURRCNTR_CTERM       (int32_t)0    //Q15(0.f)
#define     A123_Q_CURRCNTR_OUTMAX      (int32_t)31000 /* ����PWM������Ӱ�� */
#define     A123_Q_CURRCNTR_SUMMAX      (int32_t)4000000*16

//*** Velocity Control Loop Coefficients *****
#define     A123_SPEEDCNTR_PTERM        (int32_t)100000//30000//30000//4000-20000//Q15(0.06f) No1=40000,No2=50000,No3=50000,No4-6=10000
#define     A123_SPEEDCNTR_ITERM        (int32_t)1000//50//100  //Q15(0.0000f)           No1=50,No2=50,No3=50,No4-6=100
#define     A123_SPEEDCNTR_CTERM        (int32_t)0    //Q15(0.f)
#define     A123_SPEEDCNTR_OUTMAX       (int32_t)MT_MAX_TORQUE /*  */
#define     A123_SPEEDCNTR_SUMMAX       (int32_t)800000*16

//*** Position Control Loop Coefficients *****
#define     A123_POSITION_PTERM        (int32_t)400//30000//4000-20000//Q15(0.06f) No1=40000,No2=50000,No3=50000,No4-6=10000
#define     A123_POSITION_ITERM        (int32_t)1//100  //Q15(0.0000f)           No1=50,No2=50,No3=50,No4-6=100
#define     A123_POSITION_DTERM        (int32_t)200000    //Q15(0.f)
#define     A123_POSITION_CTERM        (int32_t)0    //Q15(0.f)
#define     A123_POSITION_OUTMAX       (int32_t)MT_MAX_VELOCITY /*  */
#define     A123_POSITION_SUMMAX       (int32_t)50000


void foc_param_init(uint8_t servo_id);

#endif

