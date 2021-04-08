/**
 * @file gyroscope.h
 * @author your name (you@domain.com)
 * @brief 陀螺仪
 * @version 0.1
 * @date 2021-04-08
 *
 * 零漂率 drifting_rate 计算方法: 陀螺仪开始后, 记录陀螺仪返回的角度 s (deg);
 * 静置时间 t (ms) 后陀螺仪返回的原始角度信息 e (deg), 则零漂率为 (e - s) / t
 * (deg/ms)
 *
 * @copyright Copyright (c) 2021
 *
 */

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
  __IO int16_t degree_raw; /** 角度原始量, [-32768, 32767] */
  __IO float degree;       /** 换算后的角度, [-180.0, 180.0] deg */
  __IO int16_t
      logic_degree_zero_raw; /** 逻辑零对应的角度原始量, [-32768, 32767] */
  __IO float logic_degree; /** 换算后的逻辑角度, [-180.0, 180.0] deg */
  __IO int16_t omega_raw;  /** 角速度相关, 待实现, 勿用 */
  __IO float omega;        /** 角速度相关, 待实现, 勿用 */
  uint8_t buffer[GYRO_BUFFER_SIZE]; /** 串口接收缓冲区 */
  bool new_data;                    /** 新数据标志位 */
  uint32_t zero_ticks; /** 零时刻的系统时钟, 用于消除偏移值 */
  float drifting_rate; /** 零漂率, 单位 deg/ms */
} Gyro_HandleTypeDef;

bool gyro_start(Gyro_HandleTypeDef *hgyro);
bool gyro_IRQHandler(Gyro_HandleTypeDef *hgyro);
bool gyro_set_zero(Gyro_HandleTypeDef *hgyro);
bool gyro_set_logic_zero(Gyro_HandleTypeDef *hgyro);
bool gyro_set_logic_zero_as(Gyro_HandleTypeDef *hgyro, int16_t deg_zero_raw);
float gyro_raw_to_real_deg(int16_t raw);
int16_t gyro_real_deg_to_raw(float deg);

#endif  // !__GYROSCOPE_H__
