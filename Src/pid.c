#include "pid.h"

#include "stdlib.h"
#include "tim.h"

void PID_init(PID_HandleTypeDef *pid)
{
  pid->SetSpeed = 0.0;
  pid->AltualSpeed = 0.0;
  pid->error = 0.0;
  pid->erromax = 20.0;
  pid->erro_last = 0.0;
  pid->voltage = 0.0;
  pid->integral = 0.0;
  pid->Kp = 0.01;
  pid->Ki = 0.005;
  pid->Kd = 0.01;
  pid->umax = 0.2;
  pid->umin = -0.2;
  pid->imax = 0.2;
  pid->imin = -0.2;
}

void PID_realize(PID_HandleTypeDef *pid)
{
  int index;
  pid->error = pid->SetSpeed - pid->AltualSpeed;

  if (pid->voltage >= pid->umax) {
    if (fabs(pid->error) > pid->erromax)  //�ǵ���ǰ����� #include "stdlib.h"
    {
      index = 0;
    } else {
      index = 1;
      if (pid->error < 0) {
        pid->integral += pid->error;
      }
    }

  } else if (pid->voltage < pid->umin) {
    if (fabs(pid->error > pid->erromax)) {
      index = 0;
    } else {
      index = 1;
      if (pid->error > 0) {
        pid->integral += pid->error;
      }
    }
  } else {
    if (fabs(pid->error > pid->erromax)) {
      index = 0;
    } else {
      index = 1;
      pid->integral += pid->error;
    }
  }
  float integral = pid->Ki * pid->integral;
  if (integral > pid->imax) pid->integral = pid->imax / pid->Ki;
  if (integral < pid->imin) pid->integral = pid->imin / pid->Ki;
  pid->voltage = pid->Kp * pid->error + index * pid->Ki * pid->integral +
                 pid->Kd * (pid->error - pid->erro_last);
  if (pid->voltage > 0.3) pid->voltage = 0.3;
  if (pid->voltage < -0.3) pid->voltage = -0.3;
  pid->erro_last = pid->error;
}
