
#ifndef ADC_H
#define ADC_H

/* ADC ����Ӳ�����ò��� */
//#define ADC_PER_MA   8      /* ADC��������ֵת����ʵ��mAֵ��ϵ�� adc represent how many mA */
                            /* ʵ�ʵ���ֵ=ADC����ֵ*ADC_PER_MA(��λmA) */
														
#define V_R1  100.0f          /* ĸ�߲����Ϸ�ѹ���� ��λKohm*/
#define V_R2  4.7f            /* ĸ�߲����·�ѹ���� ��λKohm */
#define ADC_RES (3.3f/4096.f) /* ADC��С�ֱ��� */
#define ADC_PER_V   (int32_t)(ADC_RES/(V_R2/(V_R1+V_R2))*32768)>>15 /* ADC��ѹ����ֵת����ʵ��Vֵ��ϵ�� */ 
                                                                    /* ʵ�ʵ�ѹֵ=ADC����ֵ*ADC_PER_V(��λV) */
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

