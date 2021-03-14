#ifndef __GYROSCOPE_H__
#define __GYROSCOPE_H__

#include "cmsis_os.h"
#include "stdbool.h"
#include "stm32f4xx_hal.h"
#define GYRO_BUFFER_SIZE 30

/**
 * @brief 陀螺仪设备 handle
 *
 */
typedef struct {
  UART_HandleTypeDef *huart;
  __IO int16_t degree_raw;
  __IO float degree;
  __IO int16_t omega_raw;
  __IO float omega;
  uint8_t buffer[GYRO_BUFFER_SIZE];
  bool new_data;
  uint32_t zero_ticks;
  float drifting_rate;
} Gyro_HandleTypeDef;

bool gyro_start(Gyro_HandleTypeDef *hgyro);
bool gyro_IRQHandler(Gyro_HandleTypeDef *hgyro);
bool gyro_set_zero(Gyro_HandleTypeDef *hgyro);
#endif  // !__GYROSCOPE_H__
