#include "libpca9685_agnostic/pca9685.hpp"
#include "platform/linux/linux_i2c_bus.hpp"
#include "platform/linux/linux_delay.hpp"

#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace {
void log_message(const std::string& message) {
    std::cout << "[PCA9685 DEMO] " << message << std::endl;
}

constexpr uint8_t kServoChannel = 0;
constexpr uint16_t kMinPulseUs = 600;
constexpr uint16_t kMaxPulseUs = 2400;
} // namespace

int main() {
    log_message("Raspberry Pi PCA9685 example starting...");

    LinuxI2CBus bus("/dev/i2c-1");
    if (!bus.is_valid()) {
        log_message("Failed to open /dev/i2c-1. Check wiring and permissions.");
        return 1;
    }

    LinuxDelay delay;
    PCA9685 pwm(bus, delay);

    if (!pwm.initialize()) {
        log_message("Failed to initialize PCA9685 device.");
        return 1;
    }

    if (!pwm.set_pwm_frequency(50.0F)) {
        log_message("Failed to set PWM frequency to 50Hz.");
        return 1;
    }

    log_message("Sweeping servo connected to channel 0.");
    std::vector<uint16_t> sweep_values = {kMinPulseUs, kMaxPulseUs, (kMinPulseUs + kMaxPulseUs) / 2};

    while (true) {
        for (auto pulse : sweep_values) {
            if (!pwm.write_microseconds(kServoChannel, pulse)) {
                log_message("Failed to write pulse width to channel.");
                return 1;
            }

            log_message("Set pulse width to " + std::to_string(pulse) + "us");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    return 0;
}