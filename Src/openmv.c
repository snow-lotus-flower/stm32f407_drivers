#include "openmv.h"

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

bool openmv_start(Openmv_HandleTypeDef *hopmv)
{
  __HAL_UART_ENABLE_IT(hopmv->huart, UART_IT_IDLE);
  hopmv->new_data = false;
  return HAL_UART_Receive_DMA(hopmv->huart, hopmv->buffer,
                              OPENMV_BUFFER_SIZE) == HAL_OK;
}

bool openmv_IRQHandler(Openmv_HandleTypeDef *hopmv)
{
  bool success = false;
  if (__HAL_UART_GET_FLAG(hopmv->huart, UART_FLAG_IDLE)) {
    __HAL_UART_CLEAR_IDLEFLAG(hopmv->huart);
    HAL_UART_DMAStopRx(hopmv->huart);
    uint16_t size =
        OPENMV_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(hopmv->huart->hdmarx);
    if (size >= 7 && hopmv->buffer[3] == '+') {
      success = true;
      hopmv->new_data = true;
      memcpy(hopmv->result, hopmv->buffer, 7);
      hopmv->result[7] = 0x00;
    }

    HAL_UART_Receive_DMA(hopmv->huart, hopmv->buffer, OPENMV_BUFFER_SIZE);
  }

  return success;
}
