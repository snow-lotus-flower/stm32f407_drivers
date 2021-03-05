#ifndef __GYROSCOPE_H__
#define __GYROSCOPE_H__

#include "stdbool.h"
#include "stm32f4xx_hal.h"
#define GYRO_BUFFER_SIZE 30

/**
 * @brief 陀螺仪设备 handle
 *
 */
typedef struct {
  UART_HandleTypeDef *huart;
  int16_t degree_raw;
  float degree;
  int16_t omega_raw;
  float omega;
  uint8_t *buffer;      /** The size should be at least GYRO_BUFFER_SIZE */
  uint16_t buffer_size; /** The size should be at least GYRO_BUFFER_SIZE */
} gyro_handle_t;

bool gyro_start(gyro_handle_t *hgyro);
bool gyro_IRQHandler(gyro_handle_t *hgyro);
bool gyro_set_zero(gyro_handle_t *hgyro)
#endif  // !__GYROSCOPE_H__
