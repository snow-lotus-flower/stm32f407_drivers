#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "stdbool.h"
#include "stm32f4xx_hal.h"

typedef struct {
  TIM_HandleTypeDef *htim;
  int16_t buffer[3];
  int16_t last_counter;
  int16_t cur_counter;
  int16_t delta;
  bool new_data;
} Encoder_HandleTypeDef;

void encoder_start(Encoder_HandleTypeDef *henc);
void encoder_update(Encoder_HandleTypeDef *henc);

#endif  // !__ENCODER_H__