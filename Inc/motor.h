#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "pwm_driver.h"
#include "stdbool.h"
#include "stm32f4xx_hal.h"

/**
 * @brief 电机旋转方向
 *
 */
typedef enum {
  MOTOR_STOP,
  MOTOR_BACKWARD,
  MOTOR_FORWARD,
  MOTOR_BRAKE
} MotorDirection;

/**
 * @brief 电机 handle
 *
 */
typedef struct {
  GPIO_TypeDef *dir1_port;
  GPIO_TypeDef *dir2_port;
  PWM_HandleTypeDef *hpwm;
  uint16_t dir1_pin;
  uint16_t dir2_pin;
  float speed;
  bool brake;
} Motor_HandleTypeDef;

void motor_speed_update(Motor_HandleTypeDef *hmtr);
#endif  // !__MOTOR_H__