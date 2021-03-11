#include "encoder.h"

void encoder_start(Encoder_HandleTypeDef *henc)
{
  HAL_TIM_Encoder_Start(henc->htim, TIM_CHANNEL_ALL);
  henc->cur_counter = __HAL_TIM_GetCounter(henc->htim);
}

void encoder_update(Encoder_HandleTypeDef *henc)
{
  henc->last_counter = henc->cur_counter;
  henc->buffer[0] = henc->buffer[1];
  henc->buffer[1] = henc->buffer[2];

  henc->cur_counter = __HAL_TIM_GetCounter(henc->htim);
  henc->buffer[2] = henc->cur_counter - henc->last_counter;
  henc->delta = (henc->buffer[0] + henc->buffer[1] + henc->buffer[2]) / 3;
  henc->new_data = true;
}