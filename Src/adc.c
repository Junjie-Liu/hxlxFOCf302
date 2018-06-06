
#include "stm32f3xx_hal.h"
#include "board.h"
#include "main.h"
#include "adc.h"
#include "foc.h"
#include "QEI.h"
#include "POS.h"
#include "mag.h"

static ADC_HandleTypeDef hadc1;
static ADC_HandleTypeDef hadc2;

void adc_init(void)
{
  
  static ADC_InjectionConfTypeDef sConfig;

  __HAL_RCC_ADC1_CLK_ENABLE();
  __HAL_RCC_ADC2_CLK_ENABLE();
  
  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  //hadc1.Init.ContinuousConvMode = DISABLE;
  //hadc1.Init.DiscontinuousConvMode = DISABLE;
  //hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
  //hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGINJECCONV_T1_CC4;
  hadc1.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  
  hadc1.Instance = ADC1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler(HAL_ADC_Init_ERROR);
  }
  
  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  //hadc2.Init.ContinuousConvMode = DISABLE;
  //hadc2.Init.DiscontinuousConvMode = DISABLE;
  //hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
  //hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGINJECCONV_T1_CC4;
  hadc2.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	
  hadc2.Instance = ADC2;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler(HAL_ADC_Init_ERROR);
  }
  
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.InjectedRank = ADC_INJECTED_RANK_1;
  sConfig.InjectedSamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.InjectedNbrOfConversion = 1;
  sConfig.InjectedDiscontinuousConvMode = ENABLE;
  sConfig.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfig.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;//ADC_EXTERNALTRIGINJECCONV_T1_TRGO2;//ADC_EXTERNALTRIGINJECCONV_T1_TRGO;//ADC_EXTERNALTRIGINJECCONV_T1_TRGO2;
  
  hadc1.Instance = ADC1;
  sConfig.InjectedChannel = ADC_CHANNEL_1;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler(HAL_ADCEx_InjectedConfigChannel_ERROR);
  }
  
  hadc2.Instance = ADC2;
  sConfig.InjectedChannel = ADC_CHANNEL_1;	
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler(HAL_ADCEx_InjectedConfigChannel_ERROR);
  }	
  
  /* enable ADC */
  hadc2.Instance = ADC2;
  __HAL_ADC_ENABLE(&hadc2);
  
  hadc1.Instance = ADC1;
  __HAL_ADC_ENABLE(&hadc1);
}

void adc_start_sample(void)
{
  /* Enable the selected ADC software conversion for injected group */
  ADC2->CR |= ADC_CR_JADSTART;	
  ADC1->CR |= ADC_CR_JADSTART;
	LED1_TOG;
}



void adc_init_offset(void)
{		
  uint16_t i = 0;
  uint16_t k = 0;
  static int32_t ia = 0;
  static int32_t ib = 0;
  
  for(i = 0; i < 1000; i++)
  {
    adc_start_sample();
    
    HAL_Delay(1);     /* waiting for sample */

		
    hadc1.Instance = ADC1;
    
    if((hadc1.Instance->ISR & ADC_FLAG_JEOC) == ADC_FLAG_JEOC)
    {
      __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC);  /* clear interrupt flag */
      
      ia += ADC_IA_VALUE;
      ib += ADC_IB_VALUE;
      k++;
    }			
  }
  
  FOC.Ia_offset = ia / k; //* FOC.ADCperI;  /* scale to mA */
  FOC.Ib_offset = ib / k; //* FOC.ADCperI;  
  
  /* Enable the ADC interrupt */
  hadc1.Instance = ADC1;
  __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOC);
  
  /* Peripheral interrupt init */
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);			
}

void ADC_Interrupt(void)
{	
	//LED1_TOG;
  hadc1.Instance = ADC1;
  
  if((hadc1.Instance->ISR & ADC_FLAG_JEOC) == ADC_FLAG_JEOC)
  {
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC);  /* clear interrupt flag */
    
    //current sample
    FOC.Ia   = -((int32_t)(ADC_IA_VALUE) - FOC.Ia_offset); //* FOC.ADCperI; /* scale to mA */
    FOC.Ib   = -((int32_t)(ADC_IB_VALUE) - FOC.Ib_offset); //* FOC.ADCperI; /* scale to mA */
		
		if( FOC.QEIthetaOk == 1)
		{    
			/* MAG PWM计算转子位置和速度 */
//			FOC.QEIcnt   = get_pwm_in_duty();                                   /* PWM 计数值 */
//			FOC.QEItheta = ((FOC.QEIcnt % PWM_IN_POLAR_COUNT) * PWM_IN_CNT_TO_THETA); /* PWM 转子角度值 */
//			FOC.QEItheta = (FOC.QEItheta + FOC.QEIthetaOffset) % 0x7FFF;        /* 修正角度误差 */
//			get_pwm_vel_pos(&FOC.QEIvel, &FOC.QEIpos);                          /* PWM 转速值 */		

			/* MAG SPI计算转子位置和速度 */
			FOC.QEIcnt   = get_mag_position();                                   /* MAG  计数值 */
			FOC.QEItheta = ((FOC.QEIcnt % MAG_POLAR_COUNT) * MAG_CNT_TO_THETA); /* PWM 转子角度值 */
			FOC.QEItheta = (FOC.QEItheta + FOC.QEIthetaOffset) % 0x7FFF;        /* 修正角度误差 */
			get_mag_vel_pos(&FOC);                          /* PWM 转速值 */				
			
			//do foc loop
			foc_loop(&FOC);

			//update pwm
			updata_pwm();			
		}
  }	
}
