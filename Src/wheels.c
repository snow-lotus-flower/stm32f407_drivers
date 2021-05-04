#include "wheels.h"

void EncoderTimerCallback(void *argument);
void PIDWheelTimerCallback(void *argument);
void SpeedCompositionTimerCallback(void *argument);
void PIDYawTimerCallback(void *argument);
void PWMSetTimerCallback(void *argument);
void PIDLaserTimerCallback(void *argument);

/**
 * @brief 符号函数 x = 0 时, 返回 0; x > 0 时, 返回 1; x < 0 时, 返回 -1;
 *
 * @param x
 * @return float
 */
float sgn(float x) { return x == 0.0 ? 0.0 : x > 0.0 ? 1.0 : -1.0; }

void all_wheels_start_encoder(AllWheels_HandleTypeDef *hawhl)
{
  encoder_start(hawhl->FL->henc);
  encoder_start(hawhl->FR->henc);
  encoder_start(hawhl->RL->henc);
  encoder_start(hawhl->RR->henc);
  if (hawhl->htim_enc == NULL) {
    hawhl->htim_enc = osTimerNew(EncoderTimerCallback, osTimerPeriodic, hawhl,
                                 &(osTimerAttr_t){.name = "encoderTimer"});
  }
  osTimerStart(hawhl->htim_enc, hawhl->tim_ticks_enc);
}

void all_wheels_start_pid_wheel(AllWheels_HandleTypeDef *hawhl)
{
  all_wheels_start_encoder(hawhl);

  PID_wheel_init(hawhl->FL->hpid);
  PID_wheel_init(hawhl->FR->hpid);
  PID_wheel_init(hawhl->RL->hpid);
  PID_wheel_init(hawhl->RR->hpid);
  if (hawhl->htim_pid_wheel == NULL) {
    hawhl->htim_pid_wheel =
        osTimerNew(PIDWheelTimerCallback, osTimerPeriodic, hawhl,
                   &(osTimerAttr_t){.name = "PIDWheelTimer"});
  }
  osTimerStart(hawhl->htim_pid_wheel, hawhl->tim_ticks_pid_wheel);
}

// void all_wheels_start_pwm_output(AllWheels_HandleTypeDef *hawhl)
// {
//   if (hawhl->htim_pwm == NULL) {
//     hawhl->htim_pwm = osTimerNew(PWMSetTimerCallback, osTimerPeriodic, hawhl,
//                                  &(osTimerAttr_t){.name = "PWMSetTimer"});
//   }
//   osTimerStart(hawhl->htim_pwm, hawhl->tim_ticks_pwm);
// }

void all_wheels_start_pid_yaw(AllWheels_HandleTypeDef *hawhl)
{
  gyro_set_logic_zero(hawhl->hgyro);

  PID_yaw_init(hawhl->hpid_yaw);

  if (hawhl->htim_pid_yaw == NULL) {
    hawhl->htim_pid_yaw =
        osTimerNew(PIDYawTimerCallback, osTimerPeriodic, hawhl,
                   &(osTimerAttr_t){.name = "PIDYawTimer"});
  }
  osTimerStart(hawhl->htim_pid_yaw, hawhl->tim_ticks_pid_yaw);
}

void all_wheels_start_pid_laser(AllWheels_HandleTypeDef *hawhl)
{
  if (hawhl->htim_pid_laser == NULL) {
    hawhl->htim_pid_laser =
        osTimerNew(PIDLaserTimerCallback, osTimerPeriodic, hawhl,
                   &(osTimerAttr_t){.name = "PIDLaserTimer"});
  }
  osTimerStart(hawhl->htim_pid_laser, hawhl->tim_ticks_pid_laser);
}

void set_laser_x_enable(AllWheels_HandleTypeDef *hawhl, bool en)
{
  if (en) {
    if (!hawhl->speed_components.laser_x_en) {
      hawhl->speed_components.laser_x_en = true;
      hawhl->speed_components.laser_x = 0.;
      PID_laser_init(hawhl->hpid_las_x);
    }
  } else {
    hawhl->speed_components.laser_x_en = false;
  }
}

void set_laser_y_enable(AllWheels_HandleTypeDef *hawhl, bool en)
{
  if (en) {
    if (!hawhl->speed_components.laser_y_en) {
      hawhl->speed_components.laser_y_en = true;
      hawhl->speed_components.laser_y = 0.;
      PID_laser_init(hawhl->hpid_las_y);
    }
  } else {
    hawhl->speed_components.laser_y_en = false;
  }
}

void laser_goto_x(AllWheels_HandleTypeDef *hawhl, float dis)
{
  set_laser_x_enable(hawhl, true);
  hawhl->hpid_las_x->SetDistance = dis;
}

void laser_goto_y(AllWheels_HandleTypeDef *hawhl, float dis)
{
  set_laser_y_enable(hawhl, true);
  hawhl->hpid_las_y->SetDistance = dis;
}

void laser_goto_xy(AllWheels_HandleTypeDef *hawhl, float x_dis, float y_dis)
{
  laser_goto_x(hawhl, x_dis);
  laser_goto_y(hawhl, y_dis);
}

void laser_disable_x(AllWheels_HandleTypeDef *hawhl)
{
  hawhl->speed_components.laser_x_en = false;
}

void laser_disable_y(AllWheels_HandleTypeDef *hawhl)
{
  hawhl->speed_components.laser_y_en = false;
}

void laser_disable_xy(AllWheels_HandleTypeDef *hawhl)
{
  laser_disable_x(hawhl);
  laser_disable_y(hawhl);
}

void all_wheels_set_speed(AllWheels_HandleTypeDef *hawhl, float x, float y,
                          float yaw)
{
  float contribute_yaw =
      (hawhl->length_separation + hawhl->width_separation) / 2.0 * yaw;
  hawhl->FL->hpid->SetSpeed = x - y - contribute_yaw;
  hawhl->FR->hpid->SetSpeed = x + y + contribute_yaw;
  hawhl->RL->hpid->SetSpeed = x + y - contribute_yaw;
  hawhl->RR->hpid->SetSpeed = x - y + contribute_yaw;
}

void all_wheels_move_xy_delta(AllWheels_HandleTypeDef *hawhl, float x, float y,
                              float speed)
{
  int16_t beginFL, beginFR, beginRL, beginRR;
  float deltaFL, deltaFR, deltaRL, deltaRR;
  float deltax, deltay;
  float speedx, speedy;
  float rad;
  bool x_cont = true, y_cont = true;

  if (x == 0.0) {
    rad = y >= 0 ? M_PI_2 : -M_PI_2;
  } else if (y == 0.0) {
    rad = x >= 0 ? 0 : M_PI;
  } else {
    rad = atan2(y, x);
  }
  speedx = speed * cos(rad);
  speedy = speed * sin(rad);

  beginFL = hawhl->FL->henc->cur_counter;
  beginFR = hawhl->FR->henc->cur_counter;
  beginRL = hawhl->RL->henc->cur_counter;
  beginRR = hawhl->RR->henc->cur_counter;

  all_wheels_set_main_speed(hawhl, speedx, speedy);

  do {
    deltaFL =
        counter_to_real_dis(hawhl, hawhl->FL->henc->cur_counter - beginFL);
    deltaFR =
        counter_to_real_dis(hawhl, hawhl->FR->henc->cur_counter - beginFR);
    deltaRL =
        counter_to_real_dis(hawhl, hawhl->RL->henc->cur_counter - beginRL);
    deltaRR =
        counter_to_real_dis(hawhl, hawhl->RR->henc->cur_counter - beginRR);
    deltax = (deltaFL + deltaFR + deltaRL + deltaRR) / 4;
    deltay = (-deltaFL + deltaFR + deltaRL - deltaRR) / 4;

    if ((deltax - x) * x >= 0) {
      all_wheels_set_main_speed(hawhl, 0, hawhl->speed_components.main_y);
      x_cont = false;
      if (y_cont && fabs(hawhl->speed_components.main_y) < 5.0) {
        hawhl->speed_components.main_y =
            5.0 * sgn(hawhl->speed_components.main_y);
      }
    }
    if ((deltay - y) * y >= 0) {
      all_wheels_set_main_speed(hawhl, hawhl->speed_components.main_x, 0);
      y_cont = false;
      if (x_cont && fabs(hawhl->speed_components.main_x) < 5.0) {
        hawhl->speed_components.main_x =
            5.0 * sgn(hawhl->speed_components.main_x);
      }
    }

    osDelay(20);
  } while (x_cont || y_cont);
}

void all_wheels_start_speed_composition(AllWheels_HandleTypeDef *hawhl)
{
  if (hawhl->htim_spd == NULL) {
    hawhl->htim_spd =
        osTimerNew(SpeedCompositionTimerCallback, osTimerPeriodic, hawhl,
                   &(osTimerAttr_t){.name = "speedCompositionTimer"});
  }
  osTimerStart(hawhl->htim_spd, hawhl->tim_ticks_spd);
}

void wheel_calc_real_speed(AllWheels_HandleTypeDef *hawhl,
                           Wheel_HandleTypeDef *hwhl)
{
  hwhl->real_speed = counter_to_real_dis(hawhl, hwhl->henc->delta) /
                     (hawhl->tim_ticks_enc * portTICK_PERIOD_MS / 1000.0);
}

float counter_to_real_dis(AllWheels_HandleTypeDef *hawhl, int16_t counter)
{
  return counter / 1320.0 * hawhl->perimeter;
}

void all_wheels_set_main_speed(AllWheels_HandleTypeDef *hawhl, float x, float y)
{
  hawhl->speed_components.main_x = x;
  hawhl->speed_components.main_y = y;
}

void PIDYawTimerCallback(void *argument)
{
  AllWheels_HandleTypeDef *hawhl = (AllWheels_HandleTypeDef *)argument;

  hawhl->hpid_yaw->ActualDeg = hawhl->hgyro->logic_degree;
  PID_yaw_realize(hawhl->hpid_yaw);
  hawhl->speed_components.gyro_yaw = hawhl->hpid_yaw->omega;
}

void PIDLaserTimerCallback(void *argument)
{
  static int16_t xlastFL, xlastFR, xlastRL, xlastRR;
  static int16_t ylastFL, ylastFR, ylastRL, ylastRR;
  float deltaFL, deltaFR, deltaRL, deltaRR;
  float deltax, deltay;

  AllWheels_HandleTypeDef *hawhl = (AllWheels_HandleTypeDef *)argument;

  if (hawhl->hlas_x->new_data) {
    hawhl->hlas_x->new_data = false;
    hawhl->hpid_las_x->ActualDistance = hawhl->hlas_x->distance;
    xlastFL = hawhl->FL->henc->cur_counter;
    xlastFR = hawhl->FR->henc->cur_counter;
    xlastRL = hawhl->RL->henc->cur_counter;
    xlastRR = hawhl->RR->henc->cur_counter;
  } else {
    deltaFL =
        counter_to_real_dis(hawhl, hawhl->FL->henc->cur_counter - xlastFL);
    deltaFR =
        counter_to_real_dis(hawhl, hawhl->FR->henc->cur_counter - xlastFR);
    deltaRL =
        counter_to_real_dis(hawhl, hawhl->RL->henc->cur_counter - xlastRL);
    deltaRR =
        counter_to_real_dis(hawhl, hawhl->RR->henc->cur_counter - xlastRR);
    deltax = (deltaFL + deltaFR + deltaRL + deltaRR) / 4;
    hawhl->hpid_las_x->ActualDistance = hawhl->hlas_x->distance - deltax;
  }

  if (hawhl->hlas_y->new_data) {
    hawhl->hlas_y->new_data = false;
    hawhl->hpid_las_y->ActualDistance = hawhl->hlas_y->distance;
    ylastFL = hawhl->FL->henc->cur_counter;
    ylastFR = hawhl->FR->henc->cur_counter;
    ylastRL = hawhl->RL->henc->cur_counter;
    ylastRR = hawhl->RR->henc->cur_counter;
  } else {
    deltaFL =
        counter_to_real_dis(hawhl, hawhl->FL->henc->cur_counter - ylastFL);
    deltaFR =
        counter_to_real_dis(hawhl, hawhl->FR->henc->cur_counter - ylastFR);
    deltaRL =
        counter_to_real_dis(hawhl, hawhl->RL->henc->cur_counter - ylastRL);
    deltaRR =
        counter_to_real_dis(hawhl, hawhl->RR->henc->cur_counter - ylastRR);
    deltay = (-deltaFL + deltaFR + deltaRL - deltaRR) / 4;
    hawhl->hpid_las_y->ActualDistance = hawhl->hlas_y->distance - deltay;
  }

  PID_laser_realize(hawhl->hpid_las_x);
  PID_laser_realize(hawhl->hpid_las_y);

  hawhl->speed_components.laser_x = hawhl->hpid_las_x->velocity;
  hawhl->speed_components.laser_y = hawhl->hpid_las_y->velocity;
}

void PIDWheelTimerCallback(void *argument)
{
  AllWheels_HandleTypeDef *hawhl = (AllWheels_HandleTypeDef *)argument;

  hawhl->FL->hpid->ActualSpeed = hawhl->FL->real_speed;
  hawhl->FR->hpid->ActualSpeed = hawhl->FR->real_speed;
  hawhl->RL->hpid->ActualSpeed = hawhl->RL->real_speed;
  hawhl->RR->hpid->ActualSpeed = hawhl->RR->real_speed;

  PID_wheel_realize(hawhl->FL->hpid);
  PID_wheel_realize(hawhl->FR->hpid);
  PID_wheel_realize(hawhl->RL->hpid);
  PID_wheel_realize(hawhl->RR->hpid);

  hawhl->FL->hmtr->speed = hawhl->FL->hpid->voltage;
  hawhl->FR->hmtr->speed = hawhl->FR->hpid->voltage;
  hawhl->RL->hmtr->speed = hawhl->RL->hpid->voltage;
  hawhl->RR->hmtr->speed = hawhl->RR->hpid->voltage;

  // motor_speed_update(hawhl->FL->hmtr);
  // motor_speed_update(hawhl->FR->hmtr);
  // motor_speed_update(hawhl->RL->hmtr);
  // motor_speed_update(hawhl->RR->hmtr);
  PWMSetTimerCallback(hawhl);
}

uint32_t delta_ticks, last_ticks;
void EncoderTimerCallback(void *argument)
{
  delta_ticks = osKernelGetTickCount() - last_ticks;
  last_ticks = delta_ticks + last_ticks;
  AllWheels_HandleTypeDef *hawhl = (AllWheels_HandleTypeDef *)argument;
  encoder_update(hawhl->FL->henc);
  encoder_update(hawhl->FR->henc);
  encoder_update(hawhl->RL->henc);
  encoder_update(hawhl->RR->henc);
  wheel_calc_real_speed(hawhl, hawhl->FL);
  wheel_calc_real_speed(hawhl, hawhl->FR);
  wheel_calc_real_speed(hawhl, hawhl->RL);
  wheel_calc_real_speed(hawhl, hawhl->RR);
}

void SpeedCompositionTimerCallback(void *argument)
{
  AllWheels_HandleTypeDef *hawhl = (AllWheels_HandleTypeDef *)argument;
  SpeedComponents *components = &hawhl->speed_components;
  all_wheels_set_speed(hawhl,
                       components->main_x + components->offset_x +
                           (components->laser_x_en ? components->laser_x : 0),
                       components->main_y + components->offset_y +
                           (components->laser_y_en ? components->laser_y : 0),
                       components->gyro_yaw);
}

void PWMSetTimerCallback(void *argument)
{
  AllWheels_HandleTypeDef *hawhl = (AllWheels_HandleTypeDef *)argument;
  static float last_FL, last_FR, last_RL, last_RR;
  static int16_t last_yaw, last_arm1, last_arm2, last_arm3, last_paw,
      last_plate;

  if (last_FL != hawhl->FL->hmtr->speed) {
    last_FL = hawhl->FL->hmtr->speed;
    motor_speed_update(hawhl->FL->hmtr);
  }
  if (last_FR != hawhl->FR->hmtr->speed) {
    last_FR = hawhl->FR->hmtr->speed;
    motor_speed_update(hawhl->FR->hmtr);
  }
  if (last_RL != hawhl->RL->hmtr->speed) {
    last_RL = hawhl->RL->hmtr->speed;
    motor_speed_update(hawhl->RL->hmtr);
  }
  if (last_RR != hawhl->RR->hmtr->speed) {
    last_RR = hawhl->RR->hmtr->speed;
    motor_speed_update(hawhl->RR->hmtr);
  }

  if (last_yaw != hawhl->hsrv_yaw->pos) {
    last_yaw = hawhl->hsrv_yaw->pos;
    servo_position_update(hawhl->hsrv_yaw);
  }
  if (last_arm1 != hawhl->hsrv_arm1->pos) {
    last_arm1 = hawhl->hsrv_arm1->pos;
    servo_position_update(hawhl->hsrv_arm1);
  }
  if (last_arm2 != hawhl->hsrv_arm2->pos) {
    last_arm2 = hawhl->hsrv_arm2->pos;
    servo_position_update(hawhl->hsrv_arm2);
  }
  if (last_arm3 != hawhl->hsrv_arm3->pos) {
    last_arm3 = hawhl->hsrv_arm3->pos;
    servo_position_update(hawhl->hsrv_arm3);
  }
  if (last_paw != hawhl->hsrv_paw->pos) {
    last_paw = hawhl->hsrv_paw->pos;
    servo_position_update(hawhl->hsrv_paw);
  }
  if (last_plate != hawhl->hsrv_plate->pos) {
    last_plate = hawhl->hsrv_plate->pos;
    servo_position_update(hawhl->hsrv_plate);
  }
}
