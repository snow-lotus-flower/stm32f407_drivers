#ifndef __OPENMV_H__
#define __OPENMV_H__

#include "stdbool.h"
#include "stm32f4xx_hal.h"
#include "string.h"

#define OPENMV_BUFFER_SIZE 30

typedef struct {
  UART_HandleTypeDef *huart;
  uint8_t buffer[OPENMV_BUFFER_SIZE];
  uint8_t result[8];
  bool new_data;
} Openmv_HandleTypeDef;

bool openmv_IRQHandler(Openmv_HandleTypeDef *hopmv);
bool openmv_start(Openmv_HandleTypeDef *hopmv);

#endif  // !__OPENMV_H__