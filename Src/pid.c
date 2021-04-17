#include "pid.h"

#include "stdlib.h"
#include "tim.h"

/** 轮速闭环 PID */
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

/**
 * @brief 轮速闭环 PID 计算
 *
 * @param pid PIDWheel handle
 */
void PID_wheel_realize(PIDWheel_HandleTypeDef *pid)
{
  int index;
  // 计算误差
  pid->error = pid->SetSpeed - pid->AltualSpeed;

  /* 抗积分饱和
    1. 判断输出值是否达到极值. 若是, 则不再增加此方向的积分;
    2. 判断误差是否大于积分开始的阈值. 若当前误差很大, 则没有必要进行积分;
    3. 判断积分项的方向是否和误差项相反. 只进行缩小误差方向上的积分.
  */
  if (pid->voltage >= pid->umax) {
    if (fabs(pid->error) > pid->erromax)  //
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

  // 计算加比例后的积分项
  float integral = pid->Ki * pid->integral;
  // 如果想让轮子停下, 且实际速度已经为0, 则对积分项清零
  if (pid->SetSpeed == 0.0 && pid->AltualSpeed == 0) pid->integral = 0;
  // 如果积分项很大, 超过了规定的阈值, 则限制在阈值内
  if (integral > pid->imax) pid->integral = pid->imax / pid->Ki;
  if (integral < pid->imin) pid->integral = pid->imin / pid->Ki;

  // 计算占空比 (执行器)
  pid->voltage = pid->Kp * pid->error + index * pid->Ki * pid->integral +
                 pid->Kd * (pid->error - pid->erro_last);

  // 限制执行器的输出阈值
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

void PID_laser_init(PIDLaser_HandleTypeDef *pid)
{
  pid->SetDistance = 0.0;
  pid->AltualDistance = 0.0;
  pid->error = 0.0;
  pid->erromax = 5.0;
  pid->erro_last = 0.0;
  pid->velocity = 0.0;
  pid->integral = 0.0;
  pid->Kp = 2;
  pid->Ki = 1;
  pid->Kd = 0.5;
  pid->max = 60.0;
  pid->min = -60.0;
  pid->imax = 10;
  pid->imin = -10;
  pid->dead_zone = 0.3;
}

void PID_laser_realize(PIDLaser_HandleTypeDef *pid)
{
  int index;
  //  pid->error = pid->SetDistance - pid->AltualDistance;
  pid->error = pid->AltualDistance - pid->SetDistance;

  if (pid->velocity >= pid->max) {
    if (fabs(pid->error) > pid->erromax) {
      index = 0;
    } else {
      index = 1;
      if (pid->error < 0) {
        pid->integral += pid->error;
      }
    }

  } else if (pid->velocity < pid->min) {
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
  if (fabs(pid->error) < pid->dead_zone) pid->integral = 0;
  float integral = pid->Ki * pid->integral;
  if (integral > pid->imax) pid->integral = pid->imax / pid->Ki;
  if (integral < pid->imin) pid->integral = pid->imin / pid->Ki;
  pid->velocity = pid->Kp * pid->error + index * pid->Ki * pid->integral +
                  pid->Kd * (pid->error - pid->erro_last);
  if (pid->velocity > pid->max) pid->velocity = pid->max;
  if (pid->velocity < pid->min) pid->velocity = pid->min;
  if (fabs(pid->error) < pid->dead_zone) pid->velocity = 0.0;
  pid->erro_last = pid->error;
}
