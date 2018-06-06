/**
  ******************************************************************************
  * @file    GPIO/GPIO_EXTI/Src/main.c 
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    13-March-2015
  * @brief   This example describes how to configure and use GPIOs through 
  *          the STM32F4xx HAL API.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "main.h"
#include "board.h"
#include "pwm.h"
#include "adc.h"
#include "foc.h"
#include "foc_param.h"
#include "QEI.h"
#include "fun.h"
#include "POS.h"
#include "mag.h"


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define SEARCH_MAG_QEI_ZERO -1
#define RELEASE_BREAK 0
#define MOTOR_READY   1
#define RUN_SERVO     2
#define SEARCH_ZERO   3
#define MC_DEBUG     10


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t ServoId = 0;    /* 伺服站号 */
uint32_t timer = 0;
int8_t SysState = RELEASE_BREAK;
uint32_t ErrorNo = 0;  /* 错误内容 */
uint32_t ShaftPosition = 0; /* GEAR轴的位置 */
int32_t  QEIposErr = 0;
int32_t  PositionErr = 0;

struct {
	int32_t d1;
	int32_t d2;
	int32_t d3;
	int32_t d4;
} acValBuffer;


char JS_RTT_UpBuffer[256];        // J-Scope RTT Buffer
int32_t  JS_RTT_Channel = 1;       // J-Scope RTT Channel

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */


	
int main(void)

{

  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();
 
  /* Configure Board */
  board_init();     /* io初始化 */
	
  /* 根据伺服站号初始化参数 */
	foc_param_init(ServoId);  /* foc参数初始化 */	
	
  /* Configure the system clock to 100 MHz */
  SystemClock_Config(); 

  pwm_init();      /* pwm初始化 */
  pwm_start();     /* pwm启动 */
	
//	EN_HALF_DRIVER;
//	
//  PWM_CCR1 = 1000;
//  PWM_CCR2 = 1500;
//  PWM_CCR3 = 2000;
//	
//	while(1){};
		
  adc_init();      /* adc初始化 */
  adc_init_offset(); /* adc偏移量初始化 */
	
	mag_init();      /* MAG绝对编码器 */
//	mag_set();       /* MAG配置 */
//	while(1){};
//	
  qei_init();      /* 增量编码器 */
	
  HAL_Delay(100);	

	EN_HALF_DRIVER;               /* enable half driver */


  /* Infinite loop */
  while (1)
  {
		switch(SysState)
		{
			case SEARCH_MAG_QEI_ZERO:
			{
				if(FOC.GEAR_MAGposOk == 0)
				{					
						if(set_mag_zero(0) == 1)  /* 清除校正数据 */
						{
							ShaftPosition = MAG_MOD - get_mag_position();
							
							if(set_mag_zero(ShaftPosition) == 1)
							{
								FOC.GEAR_MAGposOk = 1;
							}
						}
					}						
				}
				break;			
			case RELEASE_BREAK:

				if(release_break(ServoId) == 1)    /* release break */
				{					
					/* MAG SPI计算转子位置和速度 */
					FOC.QEIthetaOffset = -((get_mag_position() % MAG_POLAR_COUNT) * MAG_CNT_TO_THETA); /* 转子0度偏移量 */
					
					FOC.QEIthetaOk = 1;      /* 通知转子对齐在0度  */
					
					//FOCuser.MotorMode = POSITION_MODE; /* 切换到位置模式 */
					SysState = MC_DEBUG;              /* 进入伺服模式 */
			//		SysState = SEARCH_MAG_QEI_ZERO;
				}
				break;
				
			case SEARCH_ZERO:
				
				break;
				
			case MOTOR_READY:

				break;
			
			case RUN_SERVO:

				break;		
				
			default:
				break;
		}
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 100000000
  *            HCLK(Hz)                       = 100000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 16000000
  *            PLL_M                          = 8
  *            PLL_N                          = 200
  *            PLL_P                          = 4
  *            PLL_Q                          = 4
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler(HAL_RCC_OscConfig_ERROR);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler(HAL_RCC_ClockConfig_ERROR);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(uint32_t no)
{
  volatile uint8_t stop;
	DIS_HALF_DRIVER;  /* disable mosfet driver */
	stop_pwm();       /* stop motor pwm */
	
  ErrorNo = no;
  while(1)
  {
    stop = 1;
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
