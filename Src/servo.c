#include <servo.h>

void servo_position_update(Servo_HandleTypeDef *hsrv)
{
  // pwm_set_off_time(hsrv->hpwm, hsrv->pos + hsrv->base);
  __HAL_TIM_SetCompare(hsrv->htim, hsrv->tim_ch, hsrv->pos + hsrv->base);
}