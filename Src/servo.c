#include <servo.h>

void servo_position_update(Servo_HandleTypeDef *hsrv)
{
  pwm_set_off_time(hsrv->hpwm, hsrv->pos + hsrv->base);
}