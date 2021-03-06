#include "gyroscope.h"

static HAL_StatusTypeDef HAL_UART_DMAStopRx(UART_HandleTypeDef *huart);

/**
 * @brief 停止 UART 的 DMA 接收. 函数由 HAL_UART_DMAStop 改写而来.
 *
 * @param huart UART handle
 * @return HAL_StatusTypeDef 永远返回 HAL_OK
 */
static HAL_StatusTypeDef HAL_UART_DMAStopRx(UART_HandleTypeDef *huart)
{
  uint32_t dmarequest = 0x00U;
  /* The Lock is not implemented on this API to allow the user application
     to call the HAL UART API under callbacks HAL_UART_TxCpltCallback() /
     HAL_UART_RxCpltCallback(): when calling HAL_DMA_Abort() API the DMA TX/RX
     Transfer complete interrupt is generated and the correspond call back is
     executed HAL_UART_TxCpltCallback() / HAL_UART_RxCpltCallback()
     */

  /* Stop UART DMA Rx request if ongoing */
  dmarequest = HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAR);
  if ((huart->RxState == HAL_UART_STATE_BUSY_RX) && dmarequest) {
    CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    /* Abort the UART DMA Rx stream */
    if (huart->hdmarx != NULL) {
      HAL_DMA_Abort(huart->hdmarx);
    }
    UART_EndRxTransfer(huart);
  }

  return HAL_OK;
}

/**
 * @brief 启动陀螺仪数据获取
 *
 * @param hgyro 陀螺仪 handle
 * @return true 成功
 * @return false 失败
 */
bool gyro_start(gyro_handle_t *hgyro)
{
  if (hgyro->buffer_size < GYRO_BUFFER_SIZE) {
    return false;
  }
  __HAL_UART_ENABLE_IT(hgyro->huart, UART_IT_IDLE);
  HAL_UART_Receive_DMA(hgyro->huart, hgyro->buffer, GYRO_BUFFER_SIZE);
  return true;
}

/**
 * @brief 在 UART 中断里处理陀螺仪数据的函数, 应在 UARTx_IRQHandler 中调用
 *
 * @param hgyro 陀螺仪 handle
 * @return true 成功获取到陀螺仪数据
 * @return false 未获取到陀螺仪数据
 */
bool gyro_IRQHandler(gyro_handle_t *hgyro)
{
  bool success = false;
  if (__HAL_UART_GET_FLAG(hgyro->huart, UART_FLAG_IDLE)) {
    __HAL_UART_CLEAR_IDLEFLAG(hgyro->huart);
    HAL_UART_DMAStopRx(hgyro->huart);
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

/**
 * @brief 向陀螺仪发送置零指令
 *
 * @param hgyro 陀螺仪 handle
 * @return true 成功
 * @return false 失败
 */
bool gyro_set_zero(gyro_handle_t *hgyro)
{
  static uint8_t command[] = {0xFF, 0xAA, 0x76, 0x00, 0x00};
  if (HAL_UART_Transmit_IT(hgyro->huart, command, 5) == HAL_OK)
    return true;
  else
    return false;
}