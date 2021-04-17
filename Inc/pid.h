#ifndef __PID_H__
#define __PID_H__

#include "math.h"

/**
 * @brief 轮速闭环 PID
 *
 */
typedef struct {
  float SetSpeed;     // 目标速度, cm/s
  float AltualSpeed;  // 当前实际速度, cm/s
  float error;        // 速度误差, SetSpeed - ActualSpeed
  float erromax;      // 积分分离后, 开始进行积分的阈值
  float erro_last;    // 上一次的速度误差
  float Kp, Ki, Kd;   // 比例、积分、微分系数
  float voltage;      // 输出的占空比, [0.0, 1.0]
  float integral;     // 积分项
  float umax;         // 最大占空比
  float umin;         // 最小占空比
  float imax;         // 允许的最大积分
  float imin;         // 允许的最小积分
} PIDWheel_HandleTypeDef;

/**
 * @brief Yaw 轴 PID
 *
 */
typedef struct {
  float SetDeg;      // 目标角度值, deg
  float AltualDeg;   // 当前实际角度, deg
  float error;       // 角度误差, SetDeg - AltualDeg
  float erromax;     // 积分分离后, 开始进行积分的阈值
  float erro_last;   // 上一次的角度误差
  float Kp, Ki, Kd;  // 比例、积分、微分系数
  float omega;       // 输出的角速度值, rad/s
  float integral;    // 积分项, rad/s
  float max;         // 输出允许的最大角速度, rad/s
  float min;         // 输出允许的最小角速度, rad/s
  float imax;        // 积分项最大值, rad/s
  float imin;        // 积分项最小值, rad/s
} PIDYaw_HandleTypeDef;

/**
 * @brief 激光定位 PID
 *
 */
typedef struct {
  float SetDistance;     // 目标距离值, cm
  float AltualDistance;  // 当前实际距离, cm
  float error;           // 距离误差, SetDistance - AltualDistance
  float erromax;         // 积分分离后, 开始进行积分的阈值
  float erro_last;       // 上一次的距离误差
  float Kp, Ki, Kd;      // 比例、积分、微分系数
  float velocity;        // 输出的速度值, cm/s
  float integral;        // 积分项, cm/s
  float max;             // 输出允许的最大速度, cm/s
  float min;             // 输出允许的最小速度, cm/s
  float imax;            // 积分项最大值, cm/s
  float imin;            // 积分项最小值, cm/s
  float dead_zone;
} PIDLaser_HandleTypeDef;

void PID_wheel_init(PIDWheel_HandleTypeDef *pid);
void PID_wheel_realize(PIDWheel_HandleTypeDef *pid);
void PID_yaw_init(PIDYaw_HandleTypeDef *pid);
void PID_yaw_realize(PIDYaw_HandleTypeDef *pid);
void PID_laser_init(PIDLaser_HandleTypeDef *pid);
void PID_laser_realize(PIDLaser_HandleTypeDef *pid);

#endif /* __PID_H__ */
