#ifndef __SERVO_H__
#define __SERVO_H__

#include "pwm_driver.h"
#include "stdbool.h"
#include "stm32f4xx_hal.h"

typedef struct {
  PWM_HandleTypeDef *hpwm;
  __IO int16_t pos;
  __IO uint16_t base;
} Servo_HandleTypeDef;

void servo_position_update(Servo_HandleTypeDef *hsrv);

#endif  // !__SERVO_H__