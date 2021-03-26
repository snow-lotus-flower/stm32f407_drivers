#ifndef __PWM_DRIVER_H__
#define __PWM_DRIVER_H__

#include "pca9685.h"

#define PWM_MAX_OFF_TIME 4096

typedef struct {
  PCA9685_HandleTypeDef *hpca;
  uint8_t channel;
} PWM_HandleTypeDef;

bool pwm_init(PCA9685_HandleTypeDef *hpca);
bool pwm_set_duty_cycle(PWM_HandleTypeDef *hpwm, float duty_cycle);
bool pwm_set_off_time(PWM_HandleTypeDef *hpwm, uint16_t off_time);

#endif  // !__PWM_DRIVER_H__
