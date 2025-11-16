/**
 * @file motor_control.cpp
 * @brief Raspberry Pi PCA9685电机H桥控制示例
 * 
 * 本示例演示如何使用PCA9685驱动库通过H桥控制直流电机的转速和方向。
 * 
 * ## 功能说明
 * 
 * 1. 初始化I2C总线和PCA9685芯片
 * 2. 设置PWM频率为1kHz（适用于电机控制）
 * 3. 演示H桥方向控制：
 *    - 正转：方向A=高，方向B=低
 *    - 停止：速度=0
 *    - 反转：方向A=低，方向B=高
 * 
 * ## 硬件连接
 * 
 * ### H桥控制连接方式
 * - PCA9685模块连接到Raspberry Pi的I2C总线（/dev/i2c-1）
 * - 通道0：PWM信号（速度控制，连接到电机驱动模块的PWM输入）
 * - 通道1：方向A（正转使能，连接到电机驱动模块的方向A）
 * - 通道2：方向B（反转使能，连接到电机驱动模块的方向B）
 * 
 * ### 电机驱动模块
 * 常见的H桥驱动模块：
 * - L298N：双H桥驱动，支持正反转
 * - TB6612：双H桥驱动，支持正反转
 * 
 * ## 编译和运行
 * 
 * ```bash
 * mkdir build && cd build
 * cmake ..
 * make
 * sudo ./motor_control
 * ```
 * 
 * @note 需要root权限或i2c组权限才能访问I2C设备
 * @note 电机控制通常需要更高的PWM频率（1kHz以上），本示例使用1kHz
 * @note 根据实际硬件连接修改通道号
 */

#include "libpca9685_agnostic/pca9685.hpp"
#include "platform/linux/linux_i2c_bus.hpp"
#include "platform/linux/linux_delay.hpp"

#include <chrono>
#include <iostream>
#include <string>
#include <thread>

namespace {
/**
 * @brief 输出日志消息
 * @param message 要输出的消息
 */
void log_message(const std::string& message) {
    std::cout << "[MOTOR CONTROL] " << message << std::endl;
}

/// @brief PWM速度控制通道（连接到电机驱动模块的PWM输入）
constexpr uint8_t kSpeedChannel = 12;

/// @brief 方向控制通道A（正转使能，用于H桥控制）
constexpr uint8_t kDirectionAChannel = 0;

/// @brief 方向控制通道B（反转使能，用于H桥控制）
constexpr uint8_t kDirectionBChannel = 1;
} // namespace

int main() {
    log_message("Raspberry Pi PCA9685 Motor Control Example");
    log_message("==========================================");

    // 步骤1：创建并初始化Linux I2C总线
    LinuxI2CBus bus("/dev/i2c-1");
    if (!bus.is_valid()) {
        log_message("Failed to open /dev/i2c-1. Check wiring and permissions.");
        return 1;
    }
    log_message("I2C bus initialized successfully.");

    // 步骤2：创建延迟接口对象
    LinuxDelay delay;

    // 步骤3：创建PCA9685驱动实例（构造函数自动初始化）
    PCA9685 pwm(bus, delay);
    
    // 步骤4：检查初始化状态
    if (!pwm.is_initialized()) {
        log_message("Failed to initialize PCA9685 device.");
        return 1;
    }
    log_message("PCA9685 initialized successfully.");

    // 步骤5：设置PWM频率为1kHz（适用于电机控制）
    // 注意：电机控制通常需要更高的频率（1kHz-20kHz）
    // 频率太低会导致电机噪音和振动
    if (!pwm.set_pwm_frequency(10000.0F)) {
        log_message("Failed to set PWM frequency to 10kHz.");
        return 1;
    }
    log_message("PWM frequency set to 10kHz.");
    
    // H桥方向控制
    // 通道0：PWM速度控制
    // 通道1：方向A（正转使能）
    // 通道2：方向B（反转使能）
    
    // 步骤1：正转（方向A=高，方向B=低）
    log_message("\n1. 正转（50%速度）...");
    pwm.set_speed(kDirectionBChannel, 0.0F);         // 禁用反转（全关）
    pwm.set_speed(kSpeedChannel, 50.0F);              // 50%速度
    pwm.set_speed(kDirectionAChannel, 100.0F);        // 使能正转（全开）
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    // 步骤2：停止
    log_message("2. 停止...");
    pwm.set_speed(kSpeedChannel, 0.0F);               // 速度为0%
    pwm.set_speed(kDirectionAChannel, 0.0F);          // 方向A为0（全关）
    pwm.set_speed(kDirectionBChannel, 0.0F);          // 方向B为0（全关）
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // 步骤3：反转（方向A=低，方向B=高）
    log_message("3. 反转（70%速度）...");
    pwm.set_speed(kDirectionAChannel, 0.0F);         // 禁用正转（全关）
    pwm.set_speed(kSpeedChannel, 90.0F);              // 70%速度
    pwm.set_speed(kDirectionBChannel, 100.0F);       // 使能反转（全开）
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    // 步骤4：停止并清理
    log_message("4. 停止并清理...");
    pwm.set_speed(kSpeedChannel, 0.0F);               // 速度为0%
    pwm.set_speed(kDirectionAChannel, 0.0F);          // 方向A为0（全关）
    pwm.set_speed(kDirectionBChannel, 0.0F);          // 方向B为0（全关）
    
    log_message("\n演示完成！");

    return 0;
}

