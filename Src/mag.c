
#include "stm32f3xx_hal.h"
#include "main.h"
#include "mag.h"

uint32_t Mag_Pos = 0;  /* SPI读取绝对编码器的位置 */

static SPI_HandleTypeDef mag_spi;

void mag_init(void)
{
  MAG_SPI_CLK_ENABLE;
  
  mag_spi.Instance               = MAG_SPI;
  mag_spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; /* 4500kbps */
  mag_spi.Init.Direction         = SPI_DIRECTION_2LINES;
  mag_spi.Init.CLKPhase          = SPI_PHASE_2EDGE;//SPI_PHASE_1EDGE;
  mag_spi.Init.CLKPolarity       = SPI_POLARITY_HIGH;//SPI_POLARITY_LOW;
  mag_spi.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  mag_spi.Init.CRCPolynomial     = 7;
  mag_spi.Init.DataSize          = SPI_DATASIZE_16BIT;
  mag_spi.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  mag_spi.Init.NSS               = SPI_NSS_SOFT;
  mag_spi.Init.TIMode            = SPI_TIMODE_DISABLE;
  mag_spi.Init.Mode              = SPI_MODE_MASTER;  
	
  if(HAL_SPI_Init(&mag_spi) != HAL_OK)
  {
    Error_Handler(HAL_SPI_Init_ERROR);
  }
  
  /* Enable the Peripheral */
  __HAL_SPI_ENABLE(&mag_spi);		

}

uint8_t set_mag_zero(uint16_t pos)
{
	uint8_t olval = 0;
	uint8_t ohval = 0;
	uint8_t nlval = 0;
	uint8_t nhval = 0;	

	olval = (uint8_t)(pos & 0xFF);
	ohval = (uint8_t)((pos>>8) & 0xFF);
	
	HAL_Delay(1); /* 必须有适当的延迟 */	
	
	nlval = set_mag_register(0x00, olval); 
	nhval = set_mag_register(0x01, ohval); 	
	
	if(nlval == olval && nhval == ohval)
	{
		return 1; /* 设置成功 */
	}
		
	return 0;  /* 设置失败 */
}


uint8_t set_mag_bct(uint8_t bct)
{
	uint8_t val = 0;
	
	HAL_Delay(1); /* 必须有适当的延迟 */	
	
	val = get_mag_register(0x02); /* 检测BCT补偿值 */
	
	if(val != bct)
	{
		val = set_mag_register(0x02, bct); 
	}
	
	return val;
}

void mag_set(void)
{
	uint8_t val = 0;
	
	val = get_mag_register(0x04);

	val = get_mag_register(0x06);  /* 默认0x1C */

	if(val != 0x5C)
	{
		val = set_mag_register(0x06, 0x5C); /* 配置为0x5C */
		val = get_mag_register(0x06);  /* 读回配置后的数据 */

		/* 确认配配置成功 */
		if(val != 0x5C)
		{
			Error_Handler(MAG_MGHL_SET_ERROR); /* 磁强超范围配置错误 */
		}
	}	
	
	val = get_mag_register(0x1B) & 0xc0; 
	
	if(val != 0)
	{
		Error_Handler(MAG_MGHL_ERROR); /* 磁强超范围错误 */
	}		
}

/* spi读取转子位置 */
uint32_t get_mag_position(void)
{
	MAG_SPI_CSS_EN;
	
	MAG_SPI->DR = 0;
	while ((MAG_SPI->SR & SPI_FLAG_RXNE) == 0x00);
	Mag_Pos = MAG_SPI->DR;
	
	MAG_SPI_CSS_DIS;
	
	return Mag_Pos;
}

/* spi读取寄存器 */
uint8_t get_mag_register(uint16_t address)
{
	static int32_t i = 0;
	uint8_t val = 0;
	
	MAG_SPI_CSS_EN;
	
	MAG_SPI->DR = 0x4000|((address&0x1F)<<8); /* 寄存器地址 */
	while ((MAG_SPI->SR & SPI_FLAG_RXNE) == 0x00);
	Mag_Pos = MAG_SPI->DR;                    /* 返回转子位置 */

	MAG_SPI_CSS_DIS;

	for(i = 0; i < 100; i++){}; /* 适当的延迟 */
	
	MAG_SPI_CSS_EN;
	
	MAG_SPI->DR = 0;
	while ((MAG_SPI->SR & SPI_FLAG_RXNE) == 0x00);
	val = MAG_SPI->DR>>8;	
	
	MAG_SPI_CSS_DIS;
	
	return val;
}

/* spi写寄存器 */
uint8_t set_mag_register(uint16_t address, uint8_t val)
{
	uint8_t reval = 0;
	
	MAG_SPI_CSS_EN;
	
	MAG_SPI->DR = 0x8000|((address&0x1F)<<8)|val; /* 寄存器地址和数据内容 */
	while ((MAG_SPI->SR & SPI_FLAG_RXNE) == 0x00);
	Mag_Pos = MAG_SPI->DR;                        /* 返回转子位置 */
	
	MAG_SPI_CSS_DIS;
	
	HAL_Delay(30); /* 延迟30ms */

	MAG_SPI_CSS_EN;
	
	MAG_SPI->DR = 0;
	while ((MAG_SPI->SR & SPI_FLAG_RXNE) == 0x00);
	reval = MAG_SPI->DR>>8;	
	
	MAG_SPI_CSS_DIS;
	
	return reval;
}

