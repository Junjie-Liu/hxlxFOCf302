
#ifndef MAG_H
#define MAG_H

#include <stdint.h>
#include "board.h"

#define MAG_SPI_CLK_ENABLE __HAL_RCC_SPI2_CLK_ENABLE()
#define MAG_SPI           SPI2
#define MAG_SPI_Interrupt SPI2_IRQHandler 
#define MAG_SPI_IRQn      SPI2_IRQn
#define MAG_SPI_CSS_EN        SPI2_CSS_EN
#define MAG_SPI_CSS_DIS       SPI2_CSS_DIS

#define MAG_MAX_COUNT     65535 /*  */
#define MAG_HALF_COUNT    (MAG_MAX_COUNT>>1) /*  */

extern uint32_t mag_pos; /* ¾ø¶Ô±àÂëÆ÷µÄÎ»ÖÃ */

void mag_init(void);
void mag_set(void);
uint8_t set_mag_bct(uint8_t bct);
uint8_t set_mag_zero(uint16_t pos);
uint32_t get_mag_position(void);
uint8_t get_mag_register(uint16_t address);
uint8_t set_mag_register(uint16_t address, uint8_t val);


#endif
