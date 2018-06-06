
#include "stm32f3xx_hal.h"
#include "board.h"
#include "main.h"
#include "QEI.h"
#include "foc.h"
#include "mag.h"

int32_t  QEIthetaErrorLpK = 8;
int32_t  QEIthetaErrorLpf = 0;
int32_t  QEIthetaError = 0;
uint32_t QEIlockThetaIndex = 0;
uint32_t QEIthetaIndex = 0;

int32_t MAGposError = 0;

static TIM_HandleTypeDef qei_tim;
static TIM_HandleTypeDef vel_tim;

	
void qei_init(void)
{
    
  static TIM_Encoder_InitTypeDef qeiConfig;
  static TIM_MasterConfigTypeDef qeiMasterConfig;
  
  QEI_CLK_ENABLE;

  qei_tim.Instance = QEI_TIM;
  qei_tim.Init.Prescaler = 0;
  qei_tim.Init.CounterMode = TIM_COUNTERMODE_UP;
  qei_tim.Init.Period = QEI_MOD;
  qei_tim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  qeiConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  qeiConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  qeiConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  qeiConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  qeiConfig.IC1Filter = 4;
  qeiConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  qeiConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  qeiConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  qeiConfig.IC2Filter = 4;
  if (HAL_TIM_Encoder_Init(&qei_tim, &qeiConfig) != HAL_OK)
  {
    Error_Handler(HAL_TIM_Encoder_Init_ERROR);
  }
  
  qeiMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE; /* for trigger slave timer */
  qeiMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&qei_tim, &qeiMasterConfig) != HAL_OK)
  {
    Error_Handler(HAL_TIMEx_MasterConfigSynchronization_ERROR);
  }
  
  /* Enable the Peripheral */
  __HAL_TIM_ENABLE(&qei_tim);		
}

void qei_count_set(uint32_t count)
{
  QEI_CNT = count;
}

uint32_t get_qei_count(void)
{
  return QEI_CNT;
}

void qei_pos_set(int32_t *qpos)
{

}

void get_mag_vel_pos(FOC_t *pFOC)
{
	static uint8_t once = 0;
  static uint32_t gaps = MAG_VEL_GAPS;
  static uint32_t i = 0;
  static int32_t mag_cnt_t1 = 0;
  static int32_t vel_t1 = 0;
  static int32_t vel_t0 = 0;
	static int32_t acc_t1 = 0;
	static int32_t mag_acc_lpf = 0;
	
	//int32_t mag_cnt_t0 = get_mag_position();  /* 减少延迟 */	 
	int32_t mag_cnt_t0 = pFOC->QEIcnt;  /* 减少延迟 FOC.QEIcnt = get_mag_position(); */	 

	/* 一次初始化 */
	if(once == 0)
	{
		mag_cnt_t1 = mag_cnt_t0;
		pFOC->GEAR_MAGpos = mag_cnt_t0;
		once = 1;
	}
	
  if( ++i >= gaps )
  {
    i = 0;
		
		vel_t0 = mag_cnt_t0 - mag_cnt_t1;
		
		/* 反转越过MAG计数器0点，出现速度>0时的特殊处理 */
		if(vel_t0 > (MAG_MOD/2)) 
			vel_t0 = -(MAG_MOD - vel_t0); ;
		
		/* 正转越过MAG计数器0点，出现速度<0时的特殊处理 */
		if(vel_t0 < -(MAG_MOD/2))
			vel_t0 = MAG_MOD + vel_t0;

		/* 防止错误的速度数据 impossible vel */
		if(vel_t0 < -IMPOSSIBLE_MAG_VEL)
			vel_t0 = vel_t1;		
		
		/* 防止错误的速度数据 impossible vel */
		if(vel_t0 > IMPOSSIBLE_QEI_VEL)
			vel_t0 = vel_t1;				
    
    mag_cnt_t1 = mag_cnt_t0;
    vel_t1 = vel_t0;
		
		pFOC->GEAR_MAGvel  = vel_t0;  /* 速度 */
	  pFOC->GEAR_MAGpos += vel_t0;  /* 位置 */
		
		pFOC->GEAR_MAGacc = pFOC->GEAR_MAGvel - acc_t1;
		acc_t1 = pFOC->GEAR_MAGvel;
		
		mag_acc_lpf += (pFOC->GEAR_MAGacc - pFOC->GEAR_MAGaccLpf)*pFOC->GEAR_MAGaccLpK;
		pFOC->GEAR_MAGaccLpf = mag_acc_lpf>>15;	

		
		
		/******* 计算位置的累积误差，并进行补偿 *******/
		MAGposError = (int32_t)(pFOC->GEAR_MAGpos%MAG_MOD) - mag_cnt_t0;
		
		/* 反转越过MAG计数器0点，出现速度>0时的特殊处理 */
		if(MAGposError > (MAG_MOD/2)) 
			MAGposError = -(MAG_MOD - MAGposError);
		
		/* 正转越过MAG计数器0点，出现速度<0时的特殊处理 */
		if(MAGposError < -(MAG_MOD/2))
			MAGposError = MAG_MOD + MAGposError;	

		/* 进行补偿 */
		if((MAGposError>>3) > 0)
		{
			pFOC->GEAR_MAGpos -= 1;
		}
		else if((MAGposError>>3) < 0)
		{
			pFOC->GEAR_MAGpos += 1;
		}		
		/**********************************************/
  }
}

void QEI_CHI_Interrupt(void)
{
	static uint8_t lock_count = 0;

  /* EXTI line interrupt detected */
  if(__HAL_GPIO_EXTI_GET_IT(QEI_CHI_PIN) != RESET)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(QEI_CHI_PIN);		
		
		if(lock_count <= 5)
		{
			/* 避开上电初始化转子位置 */
			if(lock_count == 5)
				QEIlockThetaIndex = (QEI_CNT % QEI_POLAR_COUNT) * QEI_CNT_TO_THETA; /* QEI转子角度值 */;
				
			lock_count++;
		}
		else
		{
		  QEIthetaIndex = (QEI_CNT % QEI_POLAR_COUNT) * QEI_CNT_TO_THETA; /* QEI转子角度值 */;
			
			QEIthetaError += (((int32_t)QEIlockThetaIndex - QEIthetaIndex) - QEIthetaErrorLpf) * QEIthetaErrorLpK;
			QEIthetaErrorLpf = QEIthetaError>>15;
			
			/* 校正转子位置 */
			FOC.QEIthetaOffset = QEIthetaErrorLpf;
		}
  }	
}