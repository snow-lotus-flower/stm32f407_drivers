#ifndef __PWM_DRIVER_H__
#define __PWM_DRIVER_H__

#include "pca9685.h"

#define PWM_MAX_OFF_TIME 4096

typedef struct {
  pca9685_handle_t *hpca;
  uint8_t channel;
} pwm_handle_t;

bool pwm_init(pca9685_handle_t *hpca);
bool pwm_set_duty_cycle(pwm_handle_t *hpwm, float duty_cycle);
bool pwm_set_off_time(pwm_handle_t *hpwm, uint8_t off_time);

#endif  // !__PWM_DRIVER_H__