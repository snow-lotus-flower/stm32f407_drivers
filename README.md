# STM32F407 Drivers

## pca9685.h & pca9685.c

PWM 驱动板芯片 PCA9685 的驱动

## pwm_driver.h & pwm_driver.c

PWM 相关函数封装. I2C 使用默认配置即可.

### Example

先创建一个 PCA9685 的 handle:

```c
pca9685_handle_t hpca = {.i2c_handle = &hi2c1,
                           .device_address = PCA9865_I2C_DEFAULT_DEVICE_ADDRESS,
                           .inverted = false};
```

其中 `defice_address` 保留默认, `i2c_handle` 设置为 I2C 接口的 handle.

然后创建 PWM 通道的 handle:

```c
pwm_handle_t servo = {.hpca = &hpca, .channel = 0};
pwm_handle_t motor = {.hpca = &hpca, .channel = 1};
...
```

初始化 PWM 驱动板, PWM 频率为 50.0 Hz:

```c
pwm_init(&hpca);
```

设置每个通道的占空比. 有两种设置方式:

- 设置高电平所占时间, 范围 [0, `PWM_MAX_OFF_TIME`]. 其中 `PWM_MAX_OFF_TIME` 预设为 4096.
  ```c
  pwm_set_off_time(&servo, 200);
  ```
- 设置高电平所占比例, 范围 [0.0, 1.0].
  ```c
  pwm_set_duty_cycle(&motor, 0.7);
  ```

## seg_display.h / seg_display.c

通过 MAX7219 芯片驱动八位七段数码管显示数字. SPI 使用默认设置即可. 需单独设置一个 GPIO_Output 引脚作为片选端 (CS).

### Example

先创建数码管的 handle 并初始化:

```c
display_handle_t hdisp = {
  .hspi = &hspi1,
  .cs_port = CS_GPIO_Port,
  .cs_pin = CS_Pin
};
display_init(hdisp);
```

然后设置显示的内容. `size` 为 `data` 的长度. 对于 `data` 中的元素, 若为字符 `'0' ~ '9'` 或数字 `0 ~ 9`, 则显示对应的字型; 否则显示字型 '-'.

```c
uint8_t data1[] = "123+321";
uint8_t data2[] = {1, 2, 3, 10, 3, 2, 1};
// 以下两种调用方式都会显示字型 `123-321`
display_set(hdisp, data1, 7);
display_set(hdisp, data2, 7);
```

## gyroscope.h / gyroscope.c

获取陀螺仪的角度和角速度数据. 通过全双工 UART 通信, 需要开启 DMA Rx 和 UART 全局中断.

## Example

先创建陀螺仪的 handle:

```c
gyro_handle_t hgyro = {.huart = &huart1};
```

若要将陀螺仪的角度归零, 运行

```c
gyro_set_zero(&hgyro);
```

若要接收陀螺仪回传的数据, 首先改写 UARTx 的全局回调函数:

```c
/* In file stm32f4xx_it.c */

void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  if (gyro_IRQHandler(&hgyro)) {
    HAL_GPIO_TogglePin(LED_3_GPIO_Port, LED_3_Pin);
  }
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}
```

当 `gyro_IRQ_Handler` 成功解析数据时, 将返回 `true`, 否则返回 `false`.

然后在合适的位置调用

```c
gyro_start(&hgyro);
```

即可. gyro_IRQ_Handler 会把解析得到的数据存储在 `hgyro.degree` 和 `hgyro.omega` 中, 分别代表角度和角速度. 陀螺仪返回的原始信息存储在 `hgyro.degree_raw` 和 `hgyro.omega_raw` 中.
