
#ifndef FOC_PARAM_H
#define FOC_PARAM_H
	
#include <stdint.h>	
//UNIT:
//current: mA
//time: S
//resistance: ohm
//inductance: H

#define ADC_PER_MA   (32768.0f/5000);      /* ADC电流采样值转换到实际mA值的系数 adc represent how many mA */
                            /* 实际电流值=ADC采样值*ADC_PER_MA(单位mA) */

/*系统 PWM参数 */
#ifndef PWM_MOD
#define PWM_MOD        2500   /* PWM周期 */
#endif

/* MotorMode */
#define STOP_MODE      0  /* 停止 */
#define TORQUE_MODE    1  /* 扭矩模式 */
#define VELOCITY_MODE  2  /* 速度模式 */
#define POSITION_MODE  3  /* 位置模式 */
	
/* ErrorMode */	
#define UNDER_VOLTAGE  0  /* 欠压故障 */
#define OVER_VOLTAGE   1  /* 过压故障 */
#define OVER_CURRENT   2  /* 过流故障 */

//big motor
#define MT_POLAR          7//21//7//11 /* 电机极对数 pairs */
#define MT_MAX_TORQUE     30000 /* 最大电流 16384 = 5000 mA */
#define MT_MAX_VELOCITY   800 //(@1KHz)  /* 电机最大转速1000RPM/60/1000 = 每个控制周期的转速 */
                                         /* 每个控制周期的转速*MAG的每转MOD = 每个控制周期的最大转速 */

#define IDQ_LPFK   4096    /*  */ 

/* 系统运行参数 */
#define START_MODE TORQUE_MODE          /* 电机默认模式 */

/* 系统功能参数 */
#define QEI_EN           1      /* QEI 使能 if QEI enabled */
#define QEI_VEL_LPFK   8192      /* 转速滤波系数 = QEI_VEL_LPFK*2^15  */ 
#define QEI_POS_LPFK   32768    /* 转速滤波系数 = QEI_VEL_LPFK*2^15  */ 
#define MAG_ACC_LPFK   1024     /* 转速滤波系数 = MAG_ACC_LPFK*2^15  */ 
#define MAG_VEL_LPFK   4096     /* 转速滤波系数 = MAG_VEL_LPFK*2^15  */ 
#define MAG_POS_LPFK   512     /* 转速滤波系数 = MAG_VEL_LPFK*2^15  */

/* PID参数 */
#define Q15(Float_Value)	\
        ((Float_Value < 0.0f) ? (int16_t)(32768 * (Float_Value) - 0.5f) \
        : (int16_t)(32767 * (Float_Value) + 0.5f))

/* PI controllers tuning values - */
//******** D Control Loop Coefficients *******
#define     A123_D_CURRCNTR_PTERM       (int32_t)200//2000//500//Q15(0.06f)
#define     A123_D_CURRCNTR_ITERM       (int32_t)25//100//100 //Q15(0.000f)
#define     A123_D_CURRCNTR_CTERM       (int32_t)0    //Q15(0.f)
#define     A123_D_CURRCNTR_OUTMAX      (int32_t)31000 /* 考虑PWM死区的影响 */
#define     A123_D_CURRCNTR_SUMMAX      (int32_t)4000000*16

//******** Q Control Loop Coefficients *******
#define     A123_Q_CURRCNTR_PTERM       (int32_t)200//2000//500//Q15(0.06f)
#define     A123_Q_CURRCNTR_ITERM       (int32_t)25//100//100 //Q15(0.000f)
#define     A123_Q_CURRCNTR_CTERM       (int32_t)0    //Q15(0.f)
#define     A123_Q_CURRCNTR_OUTMAX      (int32_t)31000 /* 考虑PWM死区的影响 */
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

