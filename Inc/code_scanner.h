#ifndef __CODE_SCANNER_H__
#define __CODE_SCANNER_H__

#include "stdbool.h"
#include "stm32f4xx_hal.h"
#include "string.h"

#define SCANNER_BUFFER_SIZE 30

/**
 * @brief 二维码模块 handle
 *
 */
typedef struct {
  UART_HandleTypeDef *huart;
  uint8_t buffer[SCANNER_BUFFER_SIZE];
  uint8_t result[8];
  bool new_data;
} scanner_handle_t;

bool scanner_start(scanner_handle_t *hscan);
bool scanner_IRQHandler(scanner_handle_t *hscan);
#endif  // !__CODE_SCANNER_H__
