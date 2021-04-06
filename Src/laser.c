#include "laser.h"

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
    /* Disable RXNE, PE and ERR (Frame error, noise error, overrun error)
     * interrupts */
    CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
    CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

    /* In case of reception waiting for IDLE event, disable also the IDLE IE
     * interrupt source */
    if (huart->ReceptionType == HAL_UART_RECEPTION_TOIDLE) {
      CLEAR_BIT(huart->Instance->CR1, USART_CR1_IDLEIE);
    }

    /* At end of Rx process, restore huart->RxState to Ready */
    huart->RxState = HAL_UART_STATE_READY;
    huart->ReceptionType = HAL_UART_RECEPTION_STANDARD;
  }

  return HAL_OK;
}

bool laser_start(Laser_HandleTypeDef *hlas)
{
  __HAL_UART_ENABLE_IT(hlas->huart, UART_IT_IDLE);
  hlas->new_data = false;
  HAL_UART_Receive_DMA(hlas->huart, hlas->buffer, LASER_BUFFER_SIZE);
  return true;
}

bool laser_IRQHandler(Laser_HandleTypeDef *hlas)
{
  bool success = false;
  if (__HAL_UART_GET_FLAG(hlas->huart, UART_FLAG_IDLE)) {
    __HAL_UART_CLEAR_IDLEFLAG(hlas->huart);
    HAL_UART_DMAStopRx(hlas->huart);
    uint16_t count =
        sscanf((const char*)hlas->buffer, "D=%fm\r\n", &hlas->distance_raw);
    if (count == 1) {
      hlas->distance = hlas->distance_raw * 100;
      hlas->new_data = true;
      success = true;
    }
    HAL_UART_Receive_DMA(hlas->huart, hlas->buffer, LASER_BUFFER_SIZE);
  }
  return success;
}
