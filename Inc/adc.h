
#ifndef ADC_H
#define ADC_H

/* ADC 采样硬件配置参数 */
//#define ADC_PER_MA   8      /* ADC电流采样值转换到实际mA值的系数 adc represent how many mA */
                            /* 实际电流值=ADC采样值*ADC_PER_MA(单位mA) */
														
#define V_R1  100.0f          /* 母线采样上分压电阻 单位Kohm*/
#define V_R2  4.7f            /* 母线采样下分压电阻 单位Kohm */
#define ADC_RES (3.3f/4096.f) /* ADC最小分辨率 */
#define ADC_PER_V   (int32_t)(ADC_RES/(V_R2/(V_R1+V_R2))*32768)>>15 /* ADC电压采样值转换到实际V值的系数 */ 
                                                                    /* 实际电压值=ADC采样值*ADC_PER_V(单位V) */
//#define ADC_IA_VALUE ADC2->JDR1
//#define ADC_IB_VALUE ADC3->JDR1
//#define ADC_VBUS_VALUE ADC1->JDR1

#define ADC_IA_VALUE ADC1->JDR1
#define ADC_IB_VALUE ADC2->JDR1
//#define ADC_VBUS_VALUE ADC1->JDR1

#define ADC_Interrupt ADC1_2_IRQHandler

void adc_init(void);
void adc_start_sample(void);
void adc_init_offset(void);
	
#endif

