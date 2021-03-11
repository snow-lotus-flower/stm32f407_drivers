#include "pwm_driver.h"

/**
 * @brief 初始化 PWM 驱动板
 *
 * @param hpca PCA9685 驱动 handle
 * @return true 初始化成功
 * @return false 初始化失败
 */
bool pwm_init(PCA9685_HandleTypeDef *hpca)
{
  if (pca9685_init(hpca))
    return true;
  else
    return false;
}

/**
 * @brief 设置一个 PWM 通道的占空比
 *
 * @param hpwm PWM 通道 handle
 * @param duty_cycle 占空比, 在 [0.0, 1.0] 之间
 * @return true 设置成功
 * @return false 设置失败
 */
bool pwm_set_duty_cycle(PWM_HandleTypeDef *hpwm, float duty_cycle)
{
  if (duty_cycle >= 0.0 && duty_cycle <= 1.0)
    return pca9685_set_channel_duty_cycle(hpwm->hpca, hpwm->channel, duty_cycle,
                                          false);
  else
    return false;
}

/**
 * @brief 设置一个 PWM 通道的高电平时长.
 *
 * @param hpwm PWM 通道 handle
 * @param off_time 高电平时长, 在 [0, PWM_MAX_OFF_TIME] 之间
 * @return true 设置成功
 * @return false 设置失败
 */
bool pwm_set_off_time(PWM_HandleTypeDef *hpwm, uint8_t off_time)
{
  if (off_time >= 0 && off_time <= PWM_MAX_OFF_TIME)
    return pca9685_set_channel_pwm_times(hpwm->hpca, hpwm->channel, 0,
                                         off_time);
  else
    return false;
}