#include "pid.h"

#include "stdlib.h"
#include "tim.h"

void PID_wheel_init(PIDWheel_HandleTypeDef *pid)
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

void PID_wheel_realize(PIDWheel_HandleTypeDef *pid)
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
  if (pid->SetSpeed == 0.0 && pid->AltualSpeed == 0) pid->integral = 0;
  if (integral > pid->imax) pid->integral = pid->imax / pid->Ki;
  if (integral < pid->imin) pid->integral = pid->imin / pid->Ki;
  pid->voltage = pid->Kp * pid->error + index * pid->Ki * pid->integral +
                 pid->Kd * (pid->error - pid->erro_last);
  if (pid->voltage > 0.6) pid->voltage = 0.6;
  if (pid->voltage < -0.6) pid->voltage = -0.6;
  pid->erro_last = pid->error;
}

void PID_yaw_init(PIDYaw_HandleTypeDef *pid)
{
  pid->SetDeg = 0.0;
  pid->AltualDeg = 0.0;
  pid->error = 0.0;
  pid->erromax = 20.0;
  pid->erro_last = 0.0;
  pid->omega = 0.0;
  pid->integral = 0.0;
  pid->Kp = 0.1;
  pid->Ki = 0.05;
  pid->Kd = 0.01;
  pid->max = 2.0;
  pid->min = -2.0;
  pid->imax = 0.1;
  pid->imin = -0.1;
}

void PID_yaw_realize(PIDYaw_HandleTypeDef *pid)
{
  int index;
  pid->error = pid->SetDeg - pid->AltualDeg;

  if (pid->omega >= pid->max) {
    if (fabs(pid->error) > pid->erromax)  //�ǵ���ǰ����� #include "stdlib.h"
    {
      index = 0;
    } else {
      index = 1;
      if (pid->error < 0) {
        pid->integral += pid->error;
      }
    }

  } else if (pid->omega < pid->min) {
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
  index = 0;
  if (fabs(pid->error) < 0.5) pid->integral = 0;
  float integral = pid->Ki * pid->integral;
  if (integral > pid->imax) pid->integral = pid->imax / pid->Ki;
  if (integral < pid->imin) pid->integral = pid->imin / pid->Ki;
  pid->omega = pid->Kp * pid->error + index * pid->Ki * pid->integral +
               pid->Kd * (pid->error - pid->erro_last);
  if (pid->omega > pid->max) pid->omega = pid->max;
  if (pid->omega < pid->min) pid->omega = pid->min;
  if (fabs(pid->error) < 0.5) pid->omega = 0.0;
  pid->erro_last = pid->error;
}
