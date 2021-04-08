/**
 * @file encoder.h
 * @author your name (you@domain.com)
 * @brief 编码器
 * @version 0.1
 * @date 2021-04-08
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "stdbool.h"
#include "stm32f4xx_hal.h"

/**
 * @brief 编码器 handle
 *
 */
typedef struct {
  TIM_HandleTypeDef *htim;
  int16_t buffer[3];    /** 最近三次 delta 的值 */
  int16_t last_counter; /** 上一次的计数器值 */
  int16_t cur_counter;  /** 这一次的计数器值 */
  int16_t delta;        /** 计数器差值 */
  bool new_data;        /** 新数据 flag */
} Encoder_HandleTypeDef;

void encoder_start(Encoder_HandleTypeDef *henc);
void encoder_update(Encoder_HandleTypeDef *henc);

#endif  // !__ENCODER_H__