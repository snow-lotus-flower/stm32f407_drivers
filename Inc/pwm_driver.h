#ifndef __PWM_DRIVER_H__
#define __PWM_DRIVER_H__

#include "pca9685.h"

#define PWM_MAX_OFF_TIME 4096

/**
 * @brief PWM 通道 handle
 *
 */
typedef struct {
  PCA9685_HandleTypeDef *hpca; /** PCA9685 芯片 handle */
  uint8_t channel;             /** 通道数字, [0, 15] */
} PWM_HandleTypeDef;

bool pwm_init(PCA9685_HandleTypeDef *hpca);
bool pwm_set_duty_cycle(PWM_HandleTypeDef *hpwm, float duty_cycle);
bool pwm_set_off_time(PWM_HandleTypeDef *hpwm, uint16_t off_time);

#endif  // !__PWM_DRIVER_H__
