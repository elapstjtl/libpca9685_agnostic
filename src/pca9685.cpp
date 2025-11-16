/**
 * @file pca9685.cpp
 * @brief PCA9685驱动类的实现
 * 
 * 本文件实现了PCA9685芯片的所有功能函数，包括初始化、PWM控制、频率设置等。
 * 关键实现细节和算法选择说明如下。
 */

#include "libpca9685_agnostic/pca9685.hpp"

#include <algorithm>
#include <array>

namespace {
/// @brief 最大通道号（PCA9685有16个通道，编号0-15）
constexpr uint16_t kMaxChannel = 15;

/**
 * @brief 检查通道号是否有效
 * @param channel 通道号
 * @return 有效返回true，无效返回false
 * @note PCA9685有16个通道（0-15），超出范围会导致访问错误的寄存器
 */
bool check_channel(uint8_t channel) {
    return channel <= kMaxChannel;
}

/**
 * @brief 将PWM值限制在有效范围内
 * @param value 原始值
 * @return 限制后的值（0-4095）
 * @note 使用std::min确保值不超过4095（12位最大值）
 * @note 这是防御性编程，防止越界访问
 */
uint16_t clamp_value(uint16_t value) {
    return std::min<uint16_t>(value, PCA9685::kResolution - 1);
}
} // namespace

PCA9685::PCA9685(libpca9685_agnostic::I2C_Interface& i2c, libpca9685_agnostic::Delay_Interface& delay, 
                 uint8_t device_address, bool auto_init, uint8_t external_clock_prescale)
    : _i2c(i2c), _delay(delay), _address(device_address), _oscillator_frequency(kDefaultOscillatorFrequency), _initialized(false) {
    if (auto_init) {
        _initialized = initialize(external_clock_prescale);
    }
}

bool PCA9685::is_initialized() const {
    return _initialized;
}

bool PCA9685::initialize(uint8_t external_clock_prescale) {
    // 首先复位芯片，确保从已知状态开始
    if (!reset()) {
        return false;
    }

    // 根据参数选择时钟源
    if (external_clock_prescale != 0) {
        // 使用外部时钟：需要提供prescale值
        if (!set_external_clock(external_clock_prescale)) {
            return false;
        }
    } else {
        // 使用内部振荡器：默认设置PWM频率为1kHz
        // 选择1kHz的原因：这是一个通用的频率，适用于大多数应用场景
        // 对于LED调光，1kHz足够高以避免闪烁；对于舵机，用户通常会重新设置为50Hz
        if (!set_pwm_frequency(1000.0F)) {
            return false;
        }
    }

    _initialized = true;
    return true;
}

bool PCA9685::reset() {
    // 通过设置RESTART位来软件复位芯片
    // RESTART位会在设置后自动清零，这是PCA9685的硬件特性
    if (!write_register(MODE1, MODE1_RESTART)) {
        return false;
    }
    
    // 等待10ms让芯片稳定
    // 这是数据手册的要求：复位后需要等待至少500us，我们使用10ms以确保稳定
    // 使用10ms而非最小500us的原因：提供足够的安全裕量，避免时序问题
    _delay.sleep_microseconds(10'000);
    return true;
}

bool PCA9685::sleep() {
    // 读取当前MODE1寄存器值，保留其他位的状态
    uint8_t mode = 0;
    if (!read_register(MODE1, mode)) {
        return false;
    }
    
    // 设置SLEEP位（使用|=操作符，保留其他位）
    // 使用位或操作而非直接赋值的原因：保留RESTART、AI等其他位的状态
    mode |= MODE1_SLEEP;
    if (!write_register(MODE1, mode)) {
        return false;
    }
    
    // 等待5ms确保芯片进入睡眠模式
    // 这是数据手册的要求：修改MODE1后需要等待至少500us
    _delay.sleep_microseconds(5'000);
    return true;
}

bool PCA9685::wakeup() {
    // 读取当前MODE1寄存器值
    uint8_t mode = 0;
    if (!read_register(MODE1, mode)) {
        return false;
    }
    
    // 清除SLEEP位（使用&= ~操作符，保留其他位）
    // 使用位与操作而非直接赋值的原因：保留其他位的状态
    mode &= ~MODE1_SLEEP;
    return write_register(MODE1, mode);
}

bool PCA9685::set_external_clock(uint8_t prescale) {
    // 读取当前MODE1寄存器值
    uint8_t mode = 0;
    if (!read_register(MODE1, mode)) {
        return false;
    }

    // 步骤1：进入睡眠模式，清除RESTART位
    // 清除RESTART位的原因：RESTART位是只写位，读取时可能为0，清除它确保状态一致
    // 必须进入睡眠模式的原因：这是数据手册的硬件要求，修改prescale前必须睡眠
    uint8_t sleep_mode = (mode & ~MODE1_RESTART) | MODE1_SLEEP;
    if (!write_register(MODE1, sleep_mode)) {
        return false;
    }

    // 步骤2：启用外部时钟
    // 必须在睡眠模式下设置EXTCLK位，否则可能无法正常工作
    uint8_t extclk_mode = sleep_mode | MODE1_EXTCLK;
    if (!write_register(MODE1, extclk_mode)) {
        return false;
    }

    // 步骤3：设置prescale值
    // 必须在睡眠模式下设置，这是硬件要求
    if (!write_register(PRESCALE, prescale)) {
        return false;
    }

    // 等待5ms确保prescale值生效
    _delay.sleep_microseconds(5'000);

    // 步骤4：退出睡眠模式，启用RESTART和AI（自动递增）
    // 清除SLEEP位：退出睡眠模式
    // 设置RESTART位：重启PWM输出
    // 设置AI位：启用自动递增，提高连续寄存器访问效率
    uint8_t restart_mode = (extclk_mode & ~MODE1_SLEEP) | MODE1_RESTART | MODE1_AI;
    if (!write_register(MODE1, restart_mode)) {
        return false;
    }

    return true;
}

bool PCA9685::set_pwm_frequency(float frequency_hz) {
    // 限制频率范围：1Hz - 3500Hz
    // 下限1Hz：避免频率过低导致计算溢出
    // 上限3500Hz：根据数据手册，PCA9685的最大PWM频率约为3500Hz（使用最小prescale=3）
    if (frequency_hz < 1.0F) {
        frequency_hz = 1.0F;
    } else if (frequency_hz > 3500.0F) {
        frequency_hz = 3500.0F;
    }

    // 计算prescale值
    // 公式来源：frequency = oscillator_freq / (4096 * (prescale + 1))
    // 推导：prescale = (oscillator_freq / (frequency * 4096)) - 1
    // 使用 +0.5F 实现四舍五入：这是标准的浮点数四舍五入技巧
    // 减1的原因：prescale寄存器存储的是(实际分频比-1)
    float prescale_value = ((_oscillator_frequency / (frequency_hz * kResolution)) + 0.5F) - 1.0F;
    
    // 限制prescale值在有效范围内
    // 必须≥3：数据手册要求，否则PWM输出不稳定
    // 必须≤255：8位寄存器上限
    if (prescale_value < static_cast<float>(kPrescaleMin)) {
        prescale_value = static_cast<float>(kPrescaleMin);
    } else if (prescale_value > static_cast<float>(kPrescaleMax)) {
        prescale_value = static_cast<float>(kPrescaleMax);
    }

    uint8_t prescale = static_cast<uint8_t>(prescale_value);

    // 保存当前MODE1寄存器值，以便恢复
    uint8_t old_mode = 0;
    if (!read_register(MODE1, old_mode)) {
        return false;
    }

    // 步骤1：进入睡眠模式，清除RESTART位
    // 必须睡眠的原因：修改prescale寄存器的硬件要求
    uint8_t sleep_mode = (old_mode & ~MODE1_RESTART) | MODE1_SLEEP;
    if (!write_register(MODE1, sleep_mode)) {
        return false;
    }

    // 步骤2：设置prescale值
    if (!write_register(PRESCALE, prescale)) {
        return false;
    }

    // 步骤3：恢复原始MODE1值（但仍在睡眠模式）
    // 这样做的原因：保留EXTCLK、AI等其他位的状态
    if (!write_register(MODE1, old_mode)) {
        return false;
    }

    // 等待5ms确保prescale值生效
    _delay.sleep_microseconds(5'000);

    // 步骤4：退出睡眠模式，启用RESTART和AI
    // 恢复old_mode并添加RESTART和AI位
    return write_register(MODE1, old_mode | MODE1_RESTART | MODE1_AI);
}

bool PCA9685::set_output_mode(bool totem_pole) {
    uint8_t mode2 = 0;
    if (!read_register(MODE2, mode2)) {
        return false;
    }

    if (totem_pole) {
        mode2 |= MODE2_OUTDRV;
    } else {
        mode2 &= ~MODE2_OUTDRV;
    }

    return write_register(MODE2, mode2);
}

uint16_t PCA9685::get_pwm(uint8_t channel, bool return_off_time) {
    if (!check_channel(channel)) {
        return 0;
    }

    uint8_t buffer[2] = {0, 0};
    uint8_t base_register = LED0_ON_L + 4 * channel + (return_off_time ? 2 : 0);
    if (!read_registers(base_register, buffer, 2)) {
        return 0;
    }

    return static_cast<uint16_t>(buffer[0]) | (static_cast<uint16_t>(buffer[1]) << 8);
}

bool PCA9685::set_pwm(uint8_t channel, uint16_t on_tick, uint16_t off_tick) {
    if (!check_channel(channel)) {
        return false;
    }

    uint8_t buffer[4];
    buffer[0] = static_cast<uint8_t>(on_tick & 0xFF);
    buffer[1] = static_cast<uint8_t>(on_tick >> 8);
    buffer[2] = static_cast<uint8_t>(off_tick & 0xFF);
    buffer[3] = static_cast<uint8_t>(off_tick >> 8);

    return write_registers(LED0_ON_L + 4 * channel, buffer, 4);
}

bool PCA9685::set_pin(uint8_t channel, uint16_t value, bool invert) {
    // 检查通道号有效性
    if (!check_channel(channel)) {
        return false;
    }

    // 限制值在有效范围内（防御性编程）
    value = clamp_value(value);
    
    if (invert) {
        // 反转模式：value越大，输出越低
        // 边界情况处理：
        if (value == 0) {
            // value=0时，反转后应该全开：ON=4096, OFF=0
            return set_pwm(channel, FULL_ON, 0);
        }
        if (value == kResolution - 1) {
            // value=4095时，反转后应该全关：ON=0, OFF=4096
            return set_pwm(channel, 0, FULL_OFF);
        }
        // 正常情况：ON=0, OFF=(4095-value)
        // 公式推导：反转后占空比 = 1 - (value/4096) = (4096-value)/4096
        return set_pwm(channel, 0, (kResolution - 1) - value);
    }

    // 正常模式：value越大，输出越高
    // 边界情况处理：
    if (value == kResolution - 1) {
        // value=4095时，应该全开：ON=4096, OFF=0
        // 使用FULL_ON的原因：这是PCA9685的特殊值，表示始终高电平
        return set_pwm(channel, FULL_ON, 0);
    }
    if (value == 0) {
        // value=0时，应该全关：ON=0, OFF=4096
        // 使用FULL_OFF的原因：这是PCA9685的特殊值，表示始终低电平
        return set_pwm(channel, 0, FULL_OFF);
    }
    // 正常情况：ON=0, OFF=value
    // 这样设置的原因：从周期开始就输出高电平，在value时刻变为低电平
    return set_pwm(channel, 0, value);
}

bool PCA9685::set_speed(uint8_t channel, float speed_percent, bool invert) {
    // 限制速度百分比在有效范围内（防御性编程）
    if (speed_percent < 0.0F) {
        speed_percent = 0.0F;
    } else if (speed_percent > 100.0F) {
        speed_percent = 100.0F;
    }
    
    // 将百分比转换为PWM值（0-4095）
    // 公式：pwm_value = (speed_percent / 100.0) * 4095
    // 使用浮点数计算保证精度，然后四舍五入到最接近的整数
    float pwm_float = (speed_percent / 100.0F) * static_cast<float>(kResolution - 1);
    uint16_t pwm_value = static_cast<uint16_t>(pwm_float + 0.5F);  // 四舍五入
    
    // 调用set_pin()设置PWM值
    // 这样可以利用set_pin()中已有的边界处理和反转逻辑
    return set_pin(channel, pwm_value, invert);
}

bool PCA9685::write_microseconds(uint8_t channel, uint16_t microseconds) {
    // 读取当前prescale值
    // prescale=0表示读取失败（read_prescale()失败时返回0）
    uint8_t prescale = read_prescale();
    if (prescale == 0) {
        return false;
    }

    // 计算每个tick对应的微秒数
    // 公式推导：
    // 1. PWM周期 = 4096 * (prescale + 1) / oscillator_freq（秒）
    // 2. 每个tick的时间 = (prescale + 1) / oscillator_freq（秒）
    // 3. 每个tick的微秒数 = (prescale + 1) / oscillator_freq * 1'000'000
    double pulse = microseconds;
    double pulse_length = 1'000'000.0;
    pulse_length *= static_cast<double>(prescale + 1U);  // 注意：prescale+1，因为寄存器存储的是(分频比-1)
    pulse_length /= static_cast<double>(_oscillator_frequency);

    // 检查计算结果有效性（防御性编程）
    if (pulse_length <= 0.0) {
        return false;
    }

    // 将微秒数转换为tick数
    // 公式：tick = microseconds / tick_time_us
    pulse /= pulse_length;

    // 限制tick值在有效范围内
    // 使用double类型的原因：保持计算精度，避免舍入误差
    if (pulse < 0.0) {
        pulse = 0.0;
    } else if (pulse > static_cast<double>(kResolution - 1)) {
        // 限制在4095以内（12位最大值）
        pulse = static_cast<double>(kResolution - 1);
    }

    // 设置PWM：ON=0（从周期开始），OFF=pulse（在pulse时刻变低）
    // 这样设置的原因：适用于舵机控制，脉宽从周期开始计算
    return set_pwm(channel, 0, static_cast<uint16_t>(pulse));
}

uint8_t PCA9685::read_prescale() {
    uint8_t prescale = 0;
    if (!read_register(PRESCALE, prescale)) {
        return 0;
    }
    return prescale;
}

void PCA9685::set_oscillator_frequency(uint32_t frequency_hz) {
    _oscillator_frequency = frequency_hz;
}

uint32_t PCA9685::oscillator_frequency() const {
    return _oscillator_frequency;
}

bool PCA9685::write_register(uint8_t reg, uint8_t value) {
    // I2C写入格式：先发送寄存器地址，再发送数据
    // 使用缓冲区的原因：I2C接口要求一次性发送所有数据
    uint8_t buffer[2] = {reg, value};
    return _i2c.write(_address, buffer, sizeof(buffer));
}

bool PCA9685::write_registers(uint8_t reg, const uint8_t* data, size_t length) {
    // 参数验证：防御性编程
    if (data == nullptr || length == 0) {
        return false;
    }

    // 使用std::array而非C数组的原因：更安全，自动管理内存
    // 大小限制为5字节：1字节寄存器地址 + 最多4字节数据
    // 限制大小的原因：PCA9685的连续寄存器写入通常不超过4字节（如PWM的ON/OFF值）
    std::array<uint8_t, 5> buffer{};
    if (length + 1 > buffer.size()) {
        return false;
    }

    // 构建I2C写入缓冲区：寄存器地址 + 数据
    buffer[0] = reg;
    std::copy_n(data, length, buffer.begin() + 1);
    // 使用std::copy_n的原因：类型安全，避免手动循环

    return _i2c.write(_address, buffer.data(), length + 1);
}

bool PCA9685::read_register(uint8_t reg, uint8_t& value) {
    // 使用原子操作write_then_read：先写入寄存器地址，然后立即读取数据
    // 这是I2C协议的标准操作，确保写入和读取在同一个I2C事务中完成
    // 使用原子操作的原因：
    // 1. 避免两次独立操作之间被其他设备打断
    // 2. 符合I2C协议规范（使用重复起始条件Repeated Start）
    // 3. 提高可靠性和性能
    return _i2c.write_then_read(_address, &reg, 1, &value, 1);
}

bool PCA9685::read_registers(uint8_t reg, uint8_t* buffer, size_t length) {
    // 参数验证：防御性编程
    if (buffer == nullptr || length == 0) {
        return false;
    }
    
    // 使用原子操作write_then_read：先写入寄存器地址，然后立即读取多个字节
    // 这是I2C协议的标准操作，确保写入和读取在同一个I2C事务中完成
    // PCA9685支持自动递增（AI位），可以连续读取多个寄存器
    // 使用原子操作的原因：
    // 1. 避免两次独立操作之间被其他设备打断
    // 2. 符合I2C协议规范（使用重复起始条件Repeated Start）
    // 3. 提高可靠性和性能
    return _i2c.write_then_read(_address, &reg, 1, buffer, length);
}

