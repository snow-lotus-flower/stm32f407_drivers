#include "motor.h"

/**
 * @brief 设置电机的旋转防向
 *
 * @param hmtr 电机 handle
 * @param dir MotorDirection 类型的方向值
 */
static void motor_set_direction(Motor_HandleTypeDef *hmtr, MotorDirection dir)
{
  GPIO_PinState dir1_state;
  GPIO_PinState dir2_state;
  switch (dir) {
    case MOTOR_STOP:
      dir1_state = GPIO_PIN_RESET;
      dir2_state = GPIO_PIN_RESET;
      break;
    case MOTOR_BACKWARD:
      dir1_state = GPIO_PIN_SET;
      dir2_state = GPIO_PIN_RESET;
      break;
    case MOTOR_FORWARD:
      dir1_state = GPIO_PIN_RESET;
      dir2_state = GPIO_PIN_SET;
      break;
    case MOTOR_BRAKE:
    default:
      dir1_state = GPIO_PIN_SET;
      dir2_state = GPIO_PIN_SET;
      break;
  }
  HAL_GPIO_WritePin(hmtr->dir1_port, hmtr->dir1_pin, dir1_state);
  HAL_GPIO_WritePin(hmtr->dir2_port, hmtr->dir2_pin, dir2_state);
}

static void motor_set_duty_cycle(Motor_HandleTypeDef *hmtr, float duty)
{
  __HAL_TIM_SetCompare(hmtr->htim, hmtr->tim_ch,
                       (uint32_t)(hmtr->htim->Instance->ARR * duty));
}

/**
 * @brief 更新电机速度, 使新的电机速度生效
 *
 * @param hmtr 电机 handle
 */
void motor_speed_update(Motor_HandleTypeDef *hmtr)
{
  if (hmtr->brake) {
    motor_set_direction(hmtr, MOTOR_BRAKE);
    motor_set_duty_cycle(hmtr, 1.0);
  } else {
    if (hmtr->speed == 0) {
      motor_set_direction(hmtr, MOTOR_STOP);
      motor_set_duty_cycle(hmtr, 0.0);
    } else if (hmtr->speed > 0) {
      motor_set_direction(hmtr, MOTOR_FORWARD);
      motor_set_duty_cycle(hmtr, hmtr->speed);
    } else {  // hmtr->speed < 0
      motor_set_direction(hmtr, MOTOR_BACKWARD);
      motor_set_duty_cycle(hmtr, -hmtr->speed);
    }
  }
}