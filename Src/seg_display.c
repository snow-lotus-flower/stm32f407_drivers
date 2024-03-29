#include "seg_display.h"

#include "ctype.h"

#define DECODE_REG (0x09)
#define DIGIT_REG(n) ((n) + 1)
#define INTENSITY_REG (0x0A)
#define SCANLIMIT_REG (0x0B)
#define SHUTDOWN_REG (0x0C)
#define DISPLAYTEST_REG (0x0F)

static void display_transmit(Display_HandleTypeDef *hdisp, uint8_t address,
                             uint8_t value);

/**
 * @brief 向 MAX7219 发送一条指令
 *
 * @param hdisp 数码管 handle
 * @param address 寄存器地址
 * @param value 寄存器值
 */
static void display_transmit(Display_HandleTypeDef *hdisp, uint8_t address,
                             uint8_t value)
{
  uint8_t command[2] = {value, address};
  HAL_GPIO_WritePin(hdisp->cs_port, hdisp->cs_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit_DMA(hdisp->hspi, command, 1);
  while (HAL_SPI_GetState(hdisp->hspi) != HAL_SPI_STATE_READY) osDelay(1);
  HAL_GPIO_WritePin(hdisp->cs_port, hdisp->cs_pin, GPIO_PIN_SET);
}

/**
 * @brief 初始化数码管
 *
 * @param hdisp 数码管设备 handle
 */
void display_init(Display_HandleTypeDef *hdisp)
{
  // 关闭测试模式
  display_transmit(hdisp, DISPLAYTEST_REG, 0);
  // 关闭待机模式
  display_transmit(hdisp, SHUTDOWN_REG, 1);
  // 设置亮度
  display_transmit(hdisp, INTENSITY_REG, 0x0);
  // 设置显示的数码管位数
  display_transmit(hdisp, SCANLIMIT_REG, 7);
  // 打开 BCD 解码模式
  display_transmit(hdisp, DECODE_REG, 0xFF);
}

/**
 * @brief 更新数码管显示内容
 *
 * @param hdisp 数码管设备 handle
 * @param data uint_8 字符串. 最多显示前 8 个字符. 对于 data[i], 若 0 <= data[i]
 * <= 9 或 '0' <= data[i] <= '9', 则输出该数字代表的字型; 否则输出 '-' 字型.
 * @param size uint_16 字符串
 */
void display_set(Display_HandleTypeDef *hdisp, uint8_t *data, uint16_t size)
{
  uint8_t num;

  if (size > 8) size = 8;

  for (int i = 0; i < 8; ++i) {
    if (i >= size) {
      num = 0x0F;
    } else if (data[i] >= 0 && data[i] <= 9) {
      num = data[i];
    } else if (isdigit(data[i])) {
      num = data[i] - '0';
    } else if (data[i] == DISP_SYMBOL_MINUS || data[i] == '-') {
      num = 0x0A;  // '-'
    } else {
      num = 0x0F;
    }
    display_transmit(hdisp, DIGIT_REG(7 - i), num);
  }
}
