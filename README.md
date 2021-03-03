# STM32F407 Drivers

[TOC]

## pca9685.h & pca9685.c

PWM 驱动板芯片 PCA9685 的驱动

## pwm_driver.h & pwm_driver.c

PWM 相关函数封装

### Example

先创建一个 PCA9685 的 handle:

```c
pca9685_handle_t hpca = {.i2c_handle = &hi2c1,
                           .device_address = PCA9865_I2C_DEFAULT_DEVICE_ADDRESS,
                           .inverted = false};
```

其中 `defice_address` 保留默认, `i2c_handle` 设置为 SPI 接口的 handle.

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
