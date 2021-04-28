#ifndef __WHEELS_H__
#define __WHEELS_H__

#include "cmsis_os.h"
#include "code_scanner.h"
#include "encoder.h"
#include "gyroscope.h"
#include "laser.h"
#include "math.h"
#include "motor.h"
#include "openmv.h"
#include "pid.h"
#include "servo.h"

typedef struct {
  Encoder_HandleTypeDef *henc;
  Motor_HandleTypeDef *hmtr;
  PIDWheel_HandleTypeDef *hpid;
  float real_speed; /** cm/s */
} Wheel_HandleTypeDef;

typedef struct {
  GPIO_TypeDef *port;
  uint16_t pin;
} Grey_HandleTypeDef;

typedef struct {
  float x;
  float y;
  float yaw;
} Speed2D;

typedef struct {
  float main_x;
  float main_y;
  float gyro_yaw;
  float laser_x;
  float laser_y;
  bool laser_x_en;
  bool laser_y_en;
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
  Openmv_HandleTypeDef *hopmv;
  PIDYaw_HandleTypeDef *hpid_yaw;
  PIDLaser_HandleTypeDef *hpid_las_x;
  PIDLaser_HandleTypeDef *hpid_las_y;

  Laser_HandleTypeDef *hlas_x;
  Laser_HandleTypeDef *hlas_y;

  Servo_HandleTypeDef *hsrv_yaw;
  Servo_HandleTypeDef *hsrv_arm1;
  Servo_HandleTypeDef *hsrv_arm2;
  Servo_HandleTypeDef *hsrv_arm3;
  Servo_HandleTypeDef *hsrv_paw;
  Servo_HandleTypeDef *hsrv_plate;

  SpeedComponents speed_components;

  osTimerId_t htim_pid_wheel;
  uint32_t tim_ticks_pid_wheel;
  osTimerId_t htim_pid_yaw;
  uint32_t tim_ticks_pid_yaw;
  osTimerId_t htim_enc;
  uint32_t tim_ticks_enc;
  osTimerId_t htim_spd;
  uint32_t tim_ticks_spd;
  osTimerId_t htim_pwm;
  uint32_t tim_ticks_pwm;
  osTimerId_t htim_pid_laser;
  uint32_t tim_ticks_pid_laser;

  float perimeter;
  float width_separation;
  float length_separation;

  Grey_HandleTypeDef *l1;
  Grey_HandleTypeDef *l2;
  Grey_HandleTypeDef *l3;
  Grey_HandleTypeDef *l4;
  Grey_HandleTypeDef *f1;
  Grey_HandleTypeDef *f2;
  Grey_HandleTypeDef *f3;
  Grey_HandleTypeDef *f4;
} AllWheels_HandleTypeDef;

float counter_to_real_dis(AllWheels_HandleTypeDef *hawhl, int16_t counter);
void all_wheels_set_speed(AllWheels_HandleTypeDef *hawhl, float x, float y,
                          float yaw);
// void all_wheels_start_pwm_output(AllWheels_HandleTypeDef *hawhl);
void all_wheels_start_encoder(AllWheels_HandleTypeDef *hawhl);
void all_wheels_start_pid_wheel(AllWheels_HandleTypeDef *hawhl);
void all_wheels_start_pid_yaw(AllWheels_HandleTypeDef *hawhl);
void all_wheels_start_speed_composition(AllWheels_HandleTypeDef *hawhl);
void all_wheels_set_main_speed(AllWheels_HandleTypeDef *hawhl, float x,
                               float y);
void all_wheels_move_xy_delta(AllWheels_HandleTypeDef *hawhl, float x, float y,
                              float speed);

void all_wheels_start_pid_laser(AllWheels_HandleTypeDef *hawhl);
void set_laser_x_enable(AllWheels_HandleTypeDef *hawhl, bool en);
void set_laser_y_enable(AllWheels_HandleTypeDef *hawhl, bool en);
void laser_goto_x(AllWheels_HandleTypeDef *hawhl, float dis);
void laser_goto_y(AllWheels_HandleTypeDef *hawhl, float dis);
void laser_goto_xy(AllWheels_HandleTypeDef *hawhl, float x_dis, float y_dis);
void laser_disable_x(AllWheels_HandleTypeDef *hawhl);
void laser_disable_y(AllWheels_HandleTypeDef *hawhl);
void laser_disable_xy(AllWheels_HandleTypeDef *hawhl);
#endif  // !__WHEELS_H__
