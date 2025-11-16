# libpca9685_agnostic: 平台无关的PCA9685 C++驱动库

`libpca9685_agnostic` 是一个现代化的、平台无关的C++驱动库，专为NXP PCA9685 16通道、12位PWM控制器设计。其核心设计思想是可移植性、可测试性和易用性，让您可以轻松地将其集成到各种项目中，无论是Arduino、ESP32等嵌入式系统，还是Raspberry Pi等单板计算机。

[![License: BSD 3-Clause](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://en.cppreference.com/w/cpp/17)
[![CMake](https://img.shields.io/badge/CMake-3.16%2B-green.svg)](https://cmake.org/)

## 核心特性

- **平台无关**: 一次编写，处处运行。只需提供特定平台的I2C和延迟函数实现，您的应用程序代码即可在任何平台上无缝运行。
- **依赖注入**: 利用抽象接口 (`I2C_Interface`, `Delay_Interface`) 将核心逻辑与硬件相关的代码完全解耦。
- **高级API**: 提供直观的函数，如 `set_speed()` (基于百分比) 和 `write_microseconds()` (用于精确舵机控制)，将底层的寄存器操作抽象化。

## 依赖注入模式

该库实现可移植性的关键在于**依赖注入**模式。`PCA9685` 驱动不直接调用特定平台的API函数 (例如Linux的 `ioctl` 或Arduino的 `Wire.h` )，而是基于两个抽象接口进行操作：

1.  `libpca9685_agnostic::I2C_Interface`: 定义了必要的I2C通信方法 (`write`, `read`, `write_then_read`)。
2.  `libpca9685_agnostic::Delay_Interface`: 定义了一个微秒级的延迟函数 (`sleep_microseconds`)。

要在新平台上使用该驱动，您唯一需要做的就是创建实现这两个接口的具体类。这种方法隔离了所有与硬件相关的代码，使得核心驱动具有普遍兼容性。

## 快速上手 (以Raspberry Pi为例)

本示例将演示如何在Raspberry Pi上通过H桥驱动器控制一个直流电机。

### 1. 硬件连接

- 将PCA9685模块连接到Raspberry Pi的I2C总线 (通常是 `/dev/i2c-1`)。
- 将电机驱动模块的PWM输入连接到PCA9685的通道12，方向A引脚连接到通道0，方向B引脚连接到通道1。

### 2. 代码示例

以下代码片段展示了如何使用本库通过H桥驱动器控制直流电机。**该示例已在真实的Raspberry Pi和电机硬件上测试通过。**

```cpp
#include "libpca9685_agnostic/pca9685.hpp"
#include "platform/linux/linux_i2c_bus.hpp"
#include "platform/linux/linux_delay.hpp"

// 1. 定义电机控制通道
constexpr uint8_t SPEED_CHANNEL = 12;
constexpr uint8_t DIR_A_CHANNEL = 0;
constexpr uint8_t DIR_B_CHANNEL = 1;

// 2. 实例化平台接口和驱动
LinuxI2CBus bus("/dev/i2c-1");
LinuxDelay delay;
PCA9685 pwm(bus, delay);

// 3. 设置一个适合电机的高PWM频率
pwm.set_pwm_frequency(10000.0F);

// 4. 控制电机以50%速度正转
pwm.set_speed(DIR_B_CHANNEL, 0.0F);      // 方向B关闭
pwm.set_speed(DIR_A_CHANNEL, 100.0F);    // 方向A打开
pwm.set_speed(SPEED_CHANNEL, 50.0F);     // 速度设为50%
```

**更详细的用法和错误处理，请参考 [`/examples/raspberry-pi/main.cpp`](./examples/raspberry-pi/main.cpp) 目录下的完整可编译示例。**

## 移植到新平台

本库目前仅提供了**开箱即用的Linux平台实现**（已在Raspberry Pi上验证）。其平台无关的设计使得移植到其他硬件（如Arduino, ESP32, STM32等）变得非常简单。

非常欢迎社区贡献其他平台的接口实现。如果您有兴趣，只需继承并实现 `I2C_Interface` 和 `Delay_Interface` 两个抽象类，即可为新平台提供支持。

## 构建与运行

本项目使用CMake进行构建。建议从项目根目录开始操作。

1.  **克隆仓库 (如果尚未克隆):**
    ```bash
    git clone https://your-repository-url/libpca9685_agnostic.git
    cd libpca9685_agnostic
    ```

2.  **创建构建目录:**
    ```bash
    mkdir build && cd build
    ```

3.  **配置CMake:**
    -   **仅构建库:**
        ```bash
        cmake ..
        ```
    -   **构建库和示例代码:**
        ```bash
        cmake .. -DPCA9685_BUILD_EXAMPLES=ON
        ```

4.  **编译:**
    ```bash
    make
    ```

5.  **运行示例 (如果已编译):**
    可执行文件将位于 `build` 目录内。
    ```bash
    # 从build目录运行
    sudo ./examples/raspberry-pi/motor_control
    ```

## API 概览

- `PCA9685(i2c, delay, address, auto_init)`: 构造函数。
- `bool initialize()`: 如果 `auto_init` 为false，则手动初始化芯片。
- `bool is_initialized() const`: 检查芯片是否成功初始化。
- `bool set_pwm_frequency(float freq_hz)`: 为所有通道设置PWM频率 (例如，舵机50Hz，电机/LED >1kHz)。
- `bool set_speed(uint8_t channel, float speed_percent, bool invert = false)`: 根据百分比 (0.0到100.0) 设置通道输出。
- `bool write_microseconds(uint8_t channel, uint16_t us)`: 根据指定的微秒脉宽设置通道输出。是舵机精确控制的理想选择。
