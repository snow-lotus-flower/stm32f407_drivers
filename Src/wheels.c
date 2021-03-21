#include "wheels.h"

void EncoderTimerCallback(void *argument);
void PIDWheelTimerCallback(void *argument);
void SpeedCompositionTimerCallback(void *argument);
void PIDYawTimerCallback(void *argument);

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

void all_wheels_start_pid_yaw(AllWheels_HandleTypeDef *hawhl)
{
  gyro_start(hawhl->hgyro);
  osDelay(100);
  gyro_set_logic_zero(hawhl->hgyro);

  PID_yaw_init(hawhl->hpid_yaw);

  if (hawhl->htim_pid_yaw == NULL) {
    hawhl->htim_pid_yaw =
        osTimerNew(PIDYawTimerCallback, osTimerPeriodic, hawhl,
                   &(osTimerAttr_t){.name = "PIDYawTimer"});
  }
  osTimerStart(hawhl->htim_pid_yaw, hawhl->tim_ticks_pid_yaw);
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
  float deltax, deltay, endx, endy;
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
  all_wheels_start_pid_wheel(hawhl);
  all_wheels_start_pid_yaw(hawhl);

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

  hawhl->hpid_yaw->AltualDeg = hawhl->hgyro->logic_degree;
  PID_yaw_realize(hawhl->hpid_yaw);
  hawhl->speed_components.gyro_yaw = hawhl->hpid_yaw->omega;
}

void PIDWheelTimerCallback(void *argument)
{
  AllWheels_HandleTypeDef *hawhl = (AllWheels_HandleTypeDef *)argument;

  hawhl->FL->hpid->AltualSpeed = hawhl->FL->real_speed;
  hawhl->FR->hpid->AltualSpeed = hawhl->FR->real_speed;
  hawhl->RL->hpid->AltualSpeed = hawhl->RL->real_speed;
  hawhl->RR->hpid->AltualSpeed = hawhl->RR->real_speed;

  PID_wheel_realize(hawhl->FL->hpid);
  PID_wheel_realize(hawhl->FR->hpid);
  PID_wheel_realize(hawhl->RL->hpid);
  PID_wheel_realize(hawhl->RR->hpid);

  hawhl->FL->hmtr->speed = hawhl->FL->hpid->voltage;
  hawhl->FR->hmtr->speed = hawhl->FR->hpid->voltage;
  hawhl->RL->hmtr->speed = hawhl->RL->hpid->voltage;
  hawhl->RR->hmtr->speed = hawhl->RR->hpid->voltage;

  motor_speed_update(hawhl->FL->hmtr);
  motor_speed_update(hawhl->FR->hmtr);
  motor_speed_update(hawhl->RL->hmtr);
  motor_speed_update(hawhl->RR->hmtr);
}

void EncoderTimerCallback(void *argument)
{
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
  all_wheels_set_speed(
      hawhl, hawhl->speed_components.main_x + hawhl->speed_components.offset_x,
      hawhl->speed_components.main_y + hawhl->speed_components.offset_y,
      hawhl->speed_components.gyro_yaw);
}
