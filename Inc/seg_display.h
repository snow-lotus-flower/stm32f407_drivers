#ifndef __SEG_DISPLAY_H__
#define __SEG_DISPLAY_H__

#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

#define DISP_SYMBOL_MINUS (0x0A)
#define DISP_SYMBOL_NULL (0x0F)

/**
 * @brief 数码管设备 handle.
 *
 */
typedef struct {
  SPI_HandleTypeDef *hspi; /** SPI Data Size = 8 Bits */
  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;
} Display_HandleTypeDef;

void display_init(Display_HandleTypeDef *hdisp);
void display_set(Display_HandleTypeDef *hdisp, uint8_t *data, uint16_t size);

#endif  // !__SEG_DISPLAY_H__
