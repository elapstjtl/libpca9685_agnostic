# libpca9685_agnostic: A Platform-Agnostic C++ Driver for PCA9685

[简体中文](./README_zh.md)

`libpca9685_agnostic` is a modern C++, platform-independent driver for the NXP PCA9685 16-channel, 12-bit PWM controller. It's designed for portability, testability, and ease of use, making it simple to integrate into projects on various platforms, from embedded systems like Arduino and ESP32 to single-board computers like the Raspberry Pi.

[![License: BSD 3-Clause](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://en.cppreference.com/w/cpp/17)
[![CMake](https://img.shields.io/badge/CMake-3.16%2B-green.svg)](https://cmake.org/)

## Core Features

- **Platform Agnostic**: Write your application code once and run it on any platform by simply providing platform-specific implementations for I2C and delay functions.
- **Dependency Injection**: Utilizes abstract interfaces (`I2C_Interface`, `Delay_Interface`) to decouple the core logic from hardware-specific code.
- **Modern C++**: Built with C++17, ensuring type safety and modern programming practices.
- **High-Level API**: Offers intuitive functions like `set_speed()` (percentage-based) and `write_microseconds()` (for precise servo control), abstracting away low-level register manipulation.
- **Testable**: The interface-based design makes unit testing straightforward by allowing mock implementations.
- **Lightweight**: The library has a minimal footprint, suitable for resource-constrained devices.

## Dependency Injection Pattern

The key to this library's portability is its use of the **Dependency Injection** pattern. Instead of directly calling platform-specific functions (like `ioctl` on Linux or `Wire.h` on Arduino), the `PCA9685` driver operates on two abstract interfaces:

1.  `libpca9685_agnostic::I2C_Interface`: Defines the required I2C communication methods (`write`, `read`, `write_then_read`).
2.  `libpca9685_agnostic::Delay_Interface`: Defines a microsecond-level delay function (`sleep_microseconds`).

To use the driver on a new platform, you only need to create concrete classes that implement these two interfaces. This approach isolates the hardware-dependent code, making the core driver universally compatible.

## Quick Start (Raspberry Pi Example)

This example demonstrates how to control a DC motor via an H-bridge. **This example has been tested on actual Raspberry Pi and motor hardware.**

### 1. Hardware Setup

- Connect the PCA9685 module to your Raspberry Pi's I2C bus (`/dev/i2c-1`).
- Connect the motor driver's PWM input to channel 12, direction A to channel 0, and direction B to channel 1 of the PCA9685.

### 2. Code

The following snippet demonstrates the core usage of the library.

```cpp
#include "libpca9685_agnostic/pca9685.hpp"
#include "platform/linux/linux_i2c_bus.hpp"
#include "platform/linux/linux_delay.hpp"

// 1. Define motor control channels
constexpr uint8_t SPEED_CHANNEL = 12;
constexpr uint8_t DIR_A_CHANNEL = 0;
constexpr uint8_t DIR_B_CHANNEL = 1;

// 2. Instantiate platform interfaces and the driver
LinuxI2CBus bus("/dev/i2c-1");
LinuxDelay delay;
PCA9685 pwm(bus, delay);

// 3. Set a high PWM frequency suitable for motors
pwm.set_pwm_frequency(10000.0F);

// 4. Drive the motor forward at 50% speed
pwm.set_speed(DIR_B_CHANNEL, 0.0F);      // Direction B OFF
pwm.set_speed(DIR_A_CHANNEL, 100.0F);    // Direction A ON
pwm.set_speed(SPEED_CHANNEL, 50.0F);     // Speed at 50%
```

**For a complete, compilable example with error handling, please see the code in the [`/examples/raspberry-pi/main.cpp`](./examples/raspberry-pi/main.cpp) directory.**

## Porting to a New Platform

This library currently provides an **out-of-the-box implementation for Linux** (verified on Raspberry Pi). Its platform-agnostic design makes it straightforward to port to other hardware, such as Arduino, ESP32, STM32, etc.

Contributions for other platforms are highly welcome. If you are interested, you only need to inherit and implement the `I2C_Interface` and `Delay_Interface` abstract classes to add support for a new platform.

## Build and Run

This project uses CMake. It is recommended to build from the project's root directory.

1.  **Clone the repository (if you haven't already):**
    ```bash
    git clone https://your-repository-url/libpca9685_agnostic.git
    cd libpca9685_agnostic
    ```

2.  **Create a build directory:**
    ```bash
    mkdir build && cd build
    ```

3.  **Configure CMake:**
    -   **To build the library only:**
        ```bash
        cmake ..
        ```
    -   **To build the library and examples:**
        ```bash
        cmake .. -DPCA9685_BUILD_EXAMPLES=ON
        ```

4.  **Compile:**
    ```bash
    make
    ```

5.  **Run the example (if compiled):**
    The executable will be located inside the `build` directory.
    ```bash
    # Run from the 'build' directory
    sudo ./examples/raspberry-pi/motor_control
    ```

## API Overview

- `PCA9685(i2c, delay, address, auto_init)`: Constructor.
- `bool initialize()`: Manually initializes the chip if `auto_init` was false.
- `bool is_initialized() const`: Checks if the chip was successfully initialized.
- `bool set_pwm_frequency(float freq_hz)`: Sets the PWM frequency for all channels (e.g., 50 Hz for servos, >1 kHz for motors/LEDs).
- `bool set_speed(uint8_t channel, float speed_percent, bool invert = false)`: Sets a channel's output based on a percentage (0.0 to 100.0).
- `bool write_microseconds(uint8_t channel, uint16_t us)`: Sets a channel's output based on a specific pulse width in microseconds. Perfect for precise servo control.

## Contributing

Contributions are welcome!

## License

This project is licensed under the BSD 3-Clause License - see the [license.txt](./license.txt) file for details.