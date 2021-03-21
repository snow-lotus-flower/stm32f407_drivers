#ifndef __PID_H__
#define __PID_H__

#include "math.h"

typedef struct {
  float SetSpeed;     //定义CNT的值，与ARR比较确定占空比
  float AltualSpeed;  //定义实际的占空比，由此得知实际速度
  float error;        //定义偏差值
  float erromax;      //定义最大差值
  float erro_last;    //定义上一个偏差值
  float Kp, Ki, Kd;   //定义比例、积分、微分系数
  float voltage;      //定义电压值（控制执行器的变量）
  float integral;     //定义积分值
  float umax;
  float umin;
  float imax;
  float imin;
} PIDWheel_HandleTypeDef;

typedef struct {
  float SetDeg;      //定义CNT的值，与ARR比较确定占空比
  float AltualDeg;   //定义实际的占空比，由此得知实际速度
  float error;       //定义偏差值
  float erromax;     //定义最大差值
  float erro_last;   //定义上一个偏差值
  float Kp, Ki, Kd;  //定义比例、积分、微分系数
  float omega;       //定义电压值（控制执行器的变量）
  float integral;    //定义积分值
  float max;
  float min;
  float imax;
  float imin;
} PIDYaw_HandleTypeDef;

void PID_wheel_init(PIDWheel_HandleTypeDef *pid);
void PID_wheel_realize(PIDWheel_HandleTypeDef *pid);
void PID_yaw_init(PIDYaw_HandleTypeDef *pid);
void PID_yaw_realize(PIDYaw_HandleTypeDef *pid);

#endif /* __PID_H__ */
