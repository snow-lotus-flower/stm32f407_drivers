/**
 * @file laser.h
 * @author your name (you@domain.com)
 * @brief 激光测距
 * @version 0.1
 * @date 2021-04-08
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef __LASER_H__
#define __LASER_H__

#include "cmsis_os.h"
#include "stdbool.h"
#include "stdio.h"
#include "stm32f4xx_hal.h"

#define LASER_BUFFER_SIZE 30

/**
 * @brief 激光测距模块 handle
 *
 */
typedef struct {
  UART_HandleTypeDef *huart;
  __IO float distance_raw;           /** 原始距离, 单位 m */
  __IO float distance;               /** 换算成厘米的距离, 单位 cm */
  uint8_t buffer[LASER_BUFFER_SIZE]; /** 串口缓冲区 */
  bool new_data;                     /** 新数据标志位 */
} Laser_HandleTypeDef;

bool laser_start(Laser_HandleTypeDef *hlas);
bool laser_IRQHandler(Laser_HandleTypeDef *hlas);

#endif  // !__LASER_H__
