#include "code_scanner.h"

static const uint8_t SCAN_COMMAND[] = {0x7E, 0x00, 0x08, 0x01, 0x00,
                                       0x02, 0x01, 0xAB, 0xCD};
static const uint8_t RESPONSE_CODE[] = {0x02, 0x00, 0x00, 0x01,
                                        0x00, 0x33, 0x31};

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

/**
 * @brief 开始一次二维码扫描
 *
 * @param hscan 二维码模块 handle
 * @return true 成功
 * @return false 失败
 */
bool scanner_start(Scanner_HandleTypeDef *hscan)
{
  bool success = true;
  hscan->new_data = false;
  __HAL_UART_ENABLE_IT(hscan->huart, UART_IT_IDLE);
  success &= HAL_UART_DMAStopRx(hscan->huart) == HAL_OK;
  success &= HAL_UART_Receive_DMA(hscan->huart, hscan->buffer,
                                  SCANNER_BUFFER_SIZE) == HAL_OK;
  success &= HAL_UART_Transmit_DMA(hscan->huart, SCAN_COMMAND,
                                   sizeof(SCAN_COMMAND)) == HAL_OK;
  return success;
}

/**
 * @brief 二维码串口中断处理函数. 在 UARTx_IRQHandler 函数中调用此函数.
 *
 * @param hscan 二维码模块 handle
 * @return true 读取到信息
 * @return false 未读取到信息
 */
bool scanner_IRQHandler(Scanner_HandleTypeDef *hscan)
{
  bool success = false;
  if (__HAL_UART_GET_FLAG(hscan->huart, UART_FLAG_IDLE)) {
    __HAL_UART_CLEAR_IDLEFLAG(hscan->huart);
    HAL_UART_DMAStopRx(hscan->huart);
    uint16_t size =
        SCANNER_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(hscan->huart->hdmarx);
    if (size == 7 && memcmp(RESPONSE_CODE, hscan->buffer, 7) == 0) {
      HAL_UART_Receive_DMA(hscan->huart, hscan->buffer, SCANNER_BUFFER_SIZE);
    } else {
      if (size > 0 && hscan->buffer[size - 1] == '\r') {
        success = true;
        hscan->new_data = true;
        memcpy(hscan->result, hscan->buffer, size - 1 < 7 ? size - 1 : 7);
        hscan->result[size < 7 ? size : 7] = 0x00;
      }
    }
  }

  return success;
}