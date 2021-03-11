# STM32F407 Drivers

## pca9685.h & pca9685.c

PWM 驱动板芯片 PCA9685 的驱动

## pwm_driver.h & pwm_driver.c

PWM 相关函数封装. I2C 使用默认配置即可.

### Example

先创建一个 PCA9685 的 handle:

```c
PCA9685_HandleTypeDef hpca = {.i2c_handle = &hi2c1,
                           .device_address = PCA9865_I2C_DEFAULT_DEVICE_ADDRESS,
                           .inverted = false};
```

其中 `defice_address` 保留默认, `i2c_handle` 设置为 I2C 接口的 handle.

然后创建 PWM 通道的 handle:

```c
PWM_HandleTypeDef servo = {.hpca = &hpca, .channel = 0};
PWM_HandleTypeDef motor = {.hpca = &hpca, .channel = 1};
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

通过 MAX7219 芯片驱动八位七段数码管显示数字. SPI 需开启发送 DMA, 设置数据长度 `16 Bits`. 需单独设置一个 GPIO_Output 引脚作为片选端 (CS).

### Example

先创建数码管的 handle 并初始化:

```c
Display_HandleTypeDef hdisp = {
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

获取陀螺仪的角度和角速度数据. 通过全双工 UART 通信, 需要开启 DMA Rx/Tx 和 UART 全局中断.

## Example

先创建陀螺仪的 handle:

```c
Gyro_HandleTypeDef hgyro = {.huart = &huart1};
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

## code_scanner.h / code_scanner.c

GM65 二维码模块驱动. UART 需开启 DMA Rx/Tx 并开启全局中断.
GM65 模块应设置为命令触发模式, 单次读码时长无限长, 结束符 CR.

### Example

首先创建二维码模块 handle

```c
Scanner_HandleTypeDef hscan = {.huart = &huart2};
```

在相应的 `UARTx_IRQHandler` 中插入中断处理函数

```c
// In file stm32f4xx_it.c

void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
  if (scanner_IRQHandler(&hscan)) {
    HAL_GPIO_TogglePin(LED_3_GPIO_Port, LED_3_Pin);
  }
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}
```

当扫描成功时, `scanner_IRQHandler` 返回 `true`, 将 `hscan.new_data` 设置为 `true`, 且将扫描到的数据存储在 `hscan.result` 中.

一次典型的扫描过程如下:

```c
scanner_start(&hscan); // 启动一次扫描
while (!hscan.new_data) osDelay(100); // 等待扫描成功
HAL_UART_Transmit(&huart3, hscan.result, 8); // 回传扫描数据
hscan.new_data = false; // 清除标志位
```

## motor.h / motor.c

电机驱动. 对于每个电机, 需要开启两个 GPIO 输出方向信号, 一个 PWM 通道输出使能信号.

### Example

```c
Motor_HandleTypeDef hmtrFL = {
.brake = false,
.dir1_pin = M1D0_Pin,
.dir1_port = M1D0_GPIO_Port,
.dir2_pin = M1D1_Pin,
.dir2_port = M1D1_GPIO_Port,
.hpwm = &motorFL_pwm,
};
pwm_init(hmtrFL.hpwm);
hmtrFL.speed = 20.0; // cm/s
motor_set_speed(&hmtrFL);
```
