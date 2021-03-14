#ifndef __WHEELS_H__
#define __WHEELS_H__

#include "cmsis_os.h"
#include "code_scanner.h"
#include "encoder.h"
#include "gyroscope.h"
#include "math.h"
#include "motor.h"
#include "pid.h"

typedef struct {
  Encoder_HandleTypeDef *henc;
  Motor_HandleTypeDef *hmtr;
  PID_HandleTypeDef *hpid;
  float real_speed; /** cm/s */
} Wheel_HandleTypeDef;

typedef struct {
  float x;
  float y;
  float yaw;
} Speed2D;

typedef struct {
  float main_x;
  float main_y;
  float gyro_yaw;
  float offset_x;
  float offset_y;
} SpeedComponents;

typedef struct {
  Wheel_HandleTypeDef *FL;
  Wheel_HandleTypeDef *FR;
  Wheel_HandleTypeDef *RL;
  Wheel_HandleTypeDef *RR;
  Gyro_HandleTypeDef *hgyro;
  Scanner_HandleTypeDef *hscan;

  SpeedComponents speed_components;
  osTimerId_t htim_pid;
  uint32_t tim_ticks_pid;
  osTimerId_t htim_enc;
  uint32_t tim_ticks_enc;
  osTimerId_t htim_spd;
  uint32_t tim_ticks_spd;
  float perimeter;
  float width_separation;
  float length_separation;
} AllWheels_HandleTypeDef;

float calc_real_dis(AllWheels_HandleTypeDef *hawhl, int16_t counter);
void all_wheels_set_speed(AllWheels_HandleTypeDef *hawhl, float x, float y,
                          float yaw);
void all_wheels_start_encoder(AllWheels_HandleTypeDef *hawhl);
void all_wheels_start_pid(AllWheels_HandleTypeDef *hawhl);
void all_wheels_start_speed_composition(AllWheels_HandleTypeDef *hawhl);
void all_wheels_set_main_speed(AllWheels_HandleTypeDef *hawhl, float x,
                               float y);
void all_wheels_move_xy_delta(AllWheels_HandleTypeDef *hawhl, float x, float y,
                              float speed);

#endif  // !__WHEELS_H__