#ifndef __LASER_H__
#define __LASER_H__

#include "cmsis_os.h"
#include "stdbool.h"
#include "stdio.h"
#include "stm32f4xx_hal.h"

#define LASER_BUFFER_SIZE 30

typedef struct {
  UART_HandleTypeDef *huart;
  __IO float distance_raw;
  __IO float distance;
  uint8_t buffer[LASER_BUFFER_SIZE];
  bool new_data;
} Laser_HandleTypeDef;

bool laser_start(Laser_HandleTypeDef *hlas);
bool laser_IRQHandler(Laser_HandleTypeDef *hlas);

#endif  // !__LASER_H__
