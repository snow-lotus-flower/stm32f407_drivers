#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "pwm_driver.h"
#include "stdbool.h"
#include "stm32f4xx_hal.h"

/**
 * @brief 电机旋转方向枚举类型
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
  GPIO_TypeDef *dir1_port; /** 电机旋转防向 GPIO 引脚 */
  GPIO_TypeDef *dir2_port; /** 电机旋转防向 GPIO 引脚 */
  PWM_HandleTypeDef *hpwm; /** 电机 PWM handle */
  uint16_t dir1_pin;       /** 电机旋转防向 GPIO 引脚 */
  uint16_t dir2_pin;       /** 电机旋转防向 GPIO 引脚 */
  float speed;             /** 电机输出的占空比, [0, 1] */
  bool brake;              /** 是否开启刹车 */
} Motor_HandleTypeDef;

void motor_speed_update(Motor_HandleTypeDef *hmtr);
#endif  // !__MOTOR_H__