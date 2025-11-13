#include "../include/libpca9685_agnostic/pca9685.hpp"

#include <algorithm>
#include <array>

namespace {
constexpr uint16_t kMaxChannel = 15;

bool check_channel(uint8_t channel) {
    return channel <= kMaxChannel;
}

uint16_t clamp_value(uint16_t value) {
    return std::min<uint16_t>(value, PCA9685::kResolution - 1);
}
} // namespace

PCA9685::PCA9685(I2C_Interface& i2c, Delay_Interface& delay, uint8_t device_address)
    : _i2c(i2c), _delay(delay), _address(device_address), _oscillator_frequency(kDefaultOscillatorFrequency) {}

bool PCA9685::initialize(uint8_t external_clock_prescale) {
    if (!reset()) {
        return false;
    }

    if (external_clock_prescale != 0) {
        if (!set_external_clock(external_clock_prescale)) {
            return false;
        }
    } else {
        if (!set_pwm_frequency(1000.0F)) {
            return false;
        }
    }

    return true;
}

bool PCA9685::reset() {
    if (!write_register(MODE1, MODE1_RESTART)) {
        return false;
    }
    _delay.sleep_microseconds(10'000);
    return true;
}

bool PCA9685::sleep() {
    uint8_t mode = 0;
    if (!read_register(MODE1, mode)) {
        return false;
    }
    mode |= MODE1_SLEEP;
    if (!write_register(MODE1, mode)) {
        return false;
    }
    _delay.sleep_microseconds(5'000);
    return true;
}

bool PCA9685::wakeup() {
    uint8_t mode = 0;
    if (!read_register(MODE1, mode)) {
        return false;
    }
    mode &= ~MODE1_SLEEP;
    return write_register(MODE1, mode);
}

bool PCA9685::set_external_clock(uint8_t prescale) {
    uint8_t mode = 0;
    if (!read_register(MODE1, mode)) {
        return false;
    }

    uint8_t sleep_mode = (mode & ~MODE1_RESTART) | MODE1_SLEEP;
    if (!write_register(MODE1, sleep_mode)) {
        return false;
    }

    uint8_t extclk_mode = sleep_mode | MODE1_EXTCLK;
    if (!write_register(MODE1, extclk_mode)) {
        return false;
    }

    if (!write_register(PRESCALE, prescale)) {
        return false;
    }

    _delay.sleep_microseconds(5'000);

    uint8_t restart_mode = (extclk_mode & ~MODE1_SLEEP) | MODE1_RESTART | MODE1_AI;
    if (!write_register(MODE1, restart_mode)) {
        return false;
    }

    return true;
}

bool PCA9685::set_pwm_frequency(float frequency_hz) {
    if (frequency_hz < 1.0F) {
        frequency_hz = 1.0F;
    } else if (frequency_hz > 3500.0F) {
        frequency_hz = 3500.0F;
    }

    float prescale_value = ((_oscillator_frequency / (frequency_hz * kResolution)) + 0.5F) - 1.0F;
    if (prescale_value < static_cast<float>(kPrescaleMin)) {
        prescale_value = static_cast<float>(kPrescaleMin);
    } else if (prescale_value > static_cast<float>(kPrescaleMax)) {
        prescale_value = static_cast<float>(kPrescaleMax);
    }

    uint8_t prescale = static_cast<uint8_t>(prescale_value);

    uint8_t old_mode = 0;
    if (!read_register(MODE1, old_mode)) {
        return false;
    }

    uint8_t sleep_mode = (old_mode & ~MODE1_RESTART) | MODE1_SLEEP;
    if (!write_register(MODE1, sleep_mode)) {
        return false;
    }

    if (!write_register(PRESCALE, prescale)) {
        return false;
    }

    if (!write_register(MODE1, old_mode)) {
        return false;
    }

    _delay.sleep_microseconds(5'000);

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
    if (!check_channel(channel)) {
        return false;
    }

    value = clamp_value(value);
    if (invert) {
        if (value == 0) {
            return set_pwm(channel, FULL_ON, 0);
        }
        if (value == kResolution - 1) {
            return set_pwm(channel, 0, FULL_OFF);
        }
        return set_pwm(channel, 0, (kResolution - 1) - value);
    }

    if (value == kResolution - 1) {
        return set_pwm(channel, FULL_ON, 0);
    }
    if (value == 0) {
        return set_pwm(channel, 0, FULL_OFF);
    }
    return set_pwm(channel, 0, value);
}

bool PCA9685::write_microseconds(uint8_t channel, uint16_t microseconds) {
    uint8_t prescale = read_prescale();
    if (prescale == 0) {
        return false;
    }

    double pulse = microseconds;
    double pulse_length = 1'000'000.0;
    pulse_length *= static_cast<double>(prescale + 1U);
    pulse_length /= static_cast<double>(_oscillator_frequency);

    if (pulse_length <= 0.0) {
        return false;
    }

    pulse /= pulse_length;

    if (pulse < 0.0) {
        pulse = 0.0;
    } else if (pulse > static_cast<double>(kResolution - 1)) {
        pulse = static_cast<double>(kResolution - 1);
    }

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
    uint8_t buffer[2] = {reg, value};
    return _i2c.write(_address, buffer, sizeof(buffer));
}

bool PCA9685::write_registers(uint8_t reg, const uint8_t* data, size_t length) {
    if (data == nullptr || length == 0) {
        return false;
    }

    std::array<uint8_t, 5> buffer{};
    if (length + 1 > buffer.size()) {
        return false;
    }

    buffer[0] = reg;
    std::copy_n(data, length, buffer.begin() + 1);

    return _i2c.write(_address, buffer.data(), length + 1);
}

bool PCA9685::read_register(uint8_t reg, uint8_t& value) {
    if (!_i2c.write(_address, &reg, 1)) {
        return false;
    }
    if (!_i2c.read(_address, &value, 1)) {
        return false;
    }
    return true;
}

bool PCA9685::read_registers(uint8_t reg, uint8_t* buffer, size_t length) {
    if (buffer == nullptr || length == 0) {
        return false;
    }
    if (!_i2c.write(_address, &reg, 1)) {
        return false;
    }
    return _i2c.read(_address, buffer, length);
}

