
#include "stm32f3xx_hal.h"
#include "board.h"

void board_init(void)
{
  static GPIO_InitTypeDef  GPIO_InitStruct;
  
  __HAL_RCC_GPIOA_CLK_ENABLE(); /* for */
  __HAL_RCC_GPIOB_CLK_ENABLE(); /* for */	
  __HAL_RCC_GPIOC_CLK_ENABLE(); /* for */
  __HAL_RCC_GPIOD_CLK_ENABLE(); /* for */
  
  /* Configure the LED pins */
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;	
  
  GPIO_InitStruct.Pin   = LED1_PIN;
  HAL_GPIO_Init(LED1_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_SET); 	

//	GPIO_InitStruct.Pin   = LED2_PIN;
//  HAL_GPIO_Init(LED2_PORT, &GPIO_InitStruct);
//  HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, GPIO_PIN_SET); 
  
  /* Configure the HALF DRIVER SD pins */
  GPIO_InitStruct.Pin   = HALF_DRIVER_SD_PIN1;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;	
  HAL_GPIO_Init(HALF_DRIVER_SD_PORT1, &GPIO_InitStruct);
  HAL_GPIO_WritePin(HALF_DRIVER_SD_PORT1, HALF_DRIVER_SD_PIN1, GPIO_PIN_RESET);	/* set low */
	
  /* Configure the HALF DRIVER SD pins */
  GPIO_InitStruct.Pin   = HALF_DRIVER_SD_PIN2;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;	
  HAL_GPIO_Init(HALF_DRIVER_SD_PORT2, &GPIO_InitStruct);
  HAL_GPIO_WritePin(HALF_DRIVER_SD_PORT2, HALF_DRIVER_SD_PIN2, GPIO_PIN_RESET);	/* set low */

  /* Configure the HALF DRIVER SD pins */
  GPIO_InitStruct.Pin   = HALF_DRIVER_SD_PIN3;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;	
  HAL_GPIO_Init(HALF_DRIVER_SD_PORT3, &GPIO_InitStruct);
  HAL_GPIO_WritePin(HALF_DRIVER_SD_PORT3, HALF_DRIVER_SD_PIN3, GPIO_PIN_RESET);	/* set low */
	
  /* Configure the HALF DRIVER SD pins */
  GPIO_InitStruct.Pin   = HALF_DRIVER_SD_PIN4;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;	
  HAL_GPIO_Init(HALF_DRIVER_SD_PORT4, &GPIO_InitStruct);
  HAL_GPIO_WritePin(HALF_DRIVER_SD_PORT4, HALF_DRIVER_SD_PIN4, GPIO_PIN_RESET);	/* set low */

  /* Configure the HALF DRIVER SD pins */
  GPIO_InitStruct.Pin   = HALF_DRIVER_SD_PIN5;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;	
  HAL_GPIO_Init(HALF_DRIVER_SD_PORT5, &GPIO_InitStruct);
  HAL_GPIO_WritePin(HALF_DRIVER_SD_PORT5, HALF_DRIVER_SD_PIN5, GPIO_PIN_RESET);	/* set low */
  
  /* Configure the ADC pins */
  GPIO_InitStruct.Mode  = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pin   = ADC1_PIN;
  HAL_GPIO_Init(ADC1_PORT, &GPIO_InitStruct);	
  
  GPIO_InitStruct.Pin   = ADC2_PIN;
  HAL_GPIO_Init(ADC2_PORT, &GPIO_InitStruct);	
  
  /* Configure the PWM Pins */
  /* TIM1 GPIO Configuration    
  PB13    ------> TIM1_CH1N
  PB14    ------> TIM1_CH2N
  PB15    ------> TIM1_CH3N
  PA8     ------> TIM1_CH1
  PA9     ------> TIM1_CH2
  PA10    ------> TIM1_CH3 
  */
  //  GPIO_InitStruct.Pin   = PWM1N_PIN|PWM2N_PIN|PWM3N_PIN;  /* low side */
  //  GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
  //  GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
  //  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  //  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  //  HAL_GPIO_Init(PWM_N_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin   = PWM1P_PIN|PWM2P_PIN|PWM3P_PIN;   /* high side */
  GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
  HAL_GPIO_Init(PWM_P_PORT, &GPIO_InitStruct);
  
	/* Configure TIM1 PWM Break pin */
//  GPIO_InitStruct.Pin   = PWM_BREAK_PIN; 
//  GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
//  GPIO_InitStruct.Pull  = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
//  HAL_GPIO_Init(PWM_BREAK_PORT, &GPIO_InitStruct);	

	/* Congigure PWM input pins */
//  GPIO_InitStruct.Pin   = PWM_IN1_PIN|PWM_IN2_PIN;   /* high side */
//  GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
//  GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
//  HAL_GPIO_Init(PWM_IN_PORT, &GPIO_InitStruct);
	
  /* Configure the QEI pins */
  GPIO_InitStruct.Pin   = QEI_CHA_PIN; /* Channl A */
  GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(QEI_CHA_PORT, &GPIO_InitStruct);
	
  GPIO_InitStruct.Pin   = QEI_CHB_PIN; /* Channl B */
  GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(QEI_CHB_PORT, &GPIO_InitStruct);	
	
  GPIO_InitStruct.Pin   = QEI_CHI_PIN;             /* Channl index */
  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT|GPIO_MODE_IT_RISING; /* 使能外部上升沿触发模式 */
  GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = 0;  /* as gpio */
  HAL_GPIO_Init(QEI_CHI_PORT, &GPIO_InitStruct);
  
	/* SPI pins */
  /* Configure the SPI pins */
  GPIO_InitStruct.Pin   = SPI2_CLK_PIN; /*  */
  GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(SPI2_CLK_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin   = SPI2_MISO_PIN; /*  */
  GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(SPI2_MISO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin   = SPI2_MOSI_PIN; /*  */
  GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(SPI2_MOSI_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin   = SPI2_CSS_PIN; /*  */
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = 0; /* as gpio */
  HAL_GPIO_Init(SPI2_CSS_PORT, &GPIO_InitStruct);	
  HAL_GPIO_WritePin(SPI2_CSS_PORT, SPI2_CSS_PIN, GPIO_PIN_SET); /* set high, active low */
}


static void exti_io_init(void)
{
	/* 所有与ESC有关的中断必须保持相同的优先级防止相互嵌套 */
	
//  /* Enable and set EXTI Interrupt */
//  /* LAN_SYNC0 中断 */
//  HAL_NVIC_SetPriority(EXTI2_IRQn, 3, 0);
//  HAL_NVIC_EnableIRQ(EXTI2_IRQn);	
//  
//  /* Enable and set EXTI Interrupt */
//  /* LAN_SYNC1 中断 */
//  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
//  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
//  
//  /* Enable and set EXTI Interrupt */
//  /* LAN_ETH_IRQ与QEI_CHI通道公用一个中断 */
//  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
//  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

