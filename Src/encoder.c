#include "encoder.h"

/**
 * @brief 开启编码器
 *
 * @param henc 编码器 handle
 */
void encoder_start(Encoder_HandleTypeDef *henc)
{
  HAL_TIM_Encoder_Start(henc->htim, TIM_CHANNEL_ALL);
  henc->cur_counter = __HAL_TIM_GetCounter(henc->htim);
}

/**
 * @brief 记录编码器的差值
 *
 * @param henc 编码器 handle
 */
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