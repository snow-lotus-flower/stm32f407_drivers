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
  __IO int16_t logic_degree_zero_raw;
  __IO float logic_degree;
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
bool gyro_set_logic_zero(Gyro_HandleTypeDef *hgyro);
bool gyro_set_logic_zero_as(Gyro_HandleTypeDef *hgyro, int16_t deg_zero_raw);
float gyro_raw_to_real_deg(int16_t raw);
int16_t gyro_real_deg_to_raw(float deg);

#endif  // !__GYROSCOPE_H__
