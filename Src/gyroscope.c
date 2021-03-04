#include "gyroscope.h"

bool gyro_start(gyro_handle_t *hgyro)
{
  if (hgyro->buffer_size < GYRO_BUFFER_SIZE) {
    return false;
  }
  __HAL_UART_ENABLE_IT(hgyro->huart, UART_IT_IDLE);
  HAL_UART_Receive_DMA(hgyro->huart, hgyro->buffer, GYRO_BUFFER_SIZE);
  return true;
}

void gyro_stop(gyro_handle_t *hgyro) { HAL_UART_AbortReceive(hgyro->huart); }

bool gyro_IRQHandler(gyro_handle_t *hgyro)
{
  bool success = false;
  if (__HAL_UART_GET_FLAG(hgyro->huart, UART_FLAG_IDLE)) {
    __HAL_UART_CLEAR_IDLEFLAG(hgyro->huart);
    HAL_UART_DMAStop(hgyro->huart);
    uint16_t count = __HAL_DMA_GET_COUNTER(hgyro->huart->hdmarx);
    if (hgyro->buffer_size - count >= 22) {
      uint8_t checksum_omega = 0, checksum_yaw = 0;
      for (int i = 0; i < 10; ++i) {
        checksum_omega += hgyro->buffer[i];
        checksum_yaw += hgyro->buffer[11 + i];
      }
      if (hgyro->buffer[0] == 0x55 && hgyro->buffer[1] == 0x52 &&
          hgyro->buffer[10] == checksum_omega && hgyro->buffer[11] == 0x55 &&
          hgyro->buffer[12] == 0x53 && hgyro->buffer[21] == checksum_yaw) {
        uint8_t wzH = hgyro->buffer[7], wzL = hgyro->buffer[6];
        hgyro->omega_raw = (int16_t)(((uint16_t)wzH << 8) | wzL);
        hgyro->omega = hgyro->degree_raw * 2000.0 / 2e15;
        uint8_t yawH = hgyro->buffer[18], yawL = hgyro->buffer[17];
        hgyro->degree_raw = (int16_t)(((uint16_t)yawH << 8) | yawL);
        hgyro->degree = hgyro->degree_raw * 180.0 / 2e15;
        success = true;
      }
    }
    gyro_start(hgyro);
  }

  return success;
}
