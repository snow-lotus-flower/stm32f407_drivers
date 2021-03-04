#ifndef __SEG_DISPLAY_H__
#define __SEG_DISPLAY_H__

#include "stm32f4xx_hal.h"

/**
 * @brief 数码管设备 handle.
 *
 */
typedef struct {
  SPI_HandleTypeDef *hspi; /** SPI Data Size = 8 Bits */
  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;
} display_handle_t;

void display_init(display_handle_t *hdisp);
void display_set(display_handle_t *hdisp, uint8_t *data, uint16_t size);

#endif  // !__SEG_DISPLAY_H__
