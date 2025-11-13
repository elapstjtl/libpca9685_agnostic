#include "SCD4XPlus/SCD4XPlus.hpp"
#include "platform/linux/linux_i2c_bus.hpp"
#include "platform/linux/linux_delay.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <string>
#include <string_view>
#include <sstream>
#include <iomanip>

// log function for debug
void my_logger(std::string_view message) {
    std::cout << "[SCD4X++ DEBUG] " << message << std::endl;
}

int main() {
    my_logger("SCD4x C++ Library Raspberry Pi Example");

    // 1. instantiate platform-specific hardware implementation
    //    Raspberry Pi 3/4/5 I2C is usually on /dev/i2c-1
    LinuxI2CBus bus(std::string("/dev/i2c-1"));
    LinuxDelay delay;

    // check if I2C bus is successfully opened
    if (!bus.is_valid()) {
        my_logger("Failed to initialize I2C bus. Please check wiring and permissions.");
        return 1;
    }

    // 2. inject hardware implementation into our platform-independent library
    SCD4XPlus sensor(bus, delay, 0x62);

    // 3. all subsequent code is platform-independent!
    //    this code can be copied and run on STM32 or other platforms
    my_logger("Starting periodic measurements...");

    if (auto started = sensor.start_periodic_measurement(); !started) {
        std::ostringstream oss;
        oss << "Failed to start periodic measurement: " << SCD4XerrorCodeToString(started.error());
        my_logger(oss.str());
        return 1;
    }

    while (true) {
        auto ready = sensor.get_data_ready_status();
        if (!ready) {
            std::ostringstream oss;
            oss << "Failed to get data ready status: " << SCD4XerrorCodeToString(ready.error());
            my_logger(oss.str());
            return 1;
        }

        if (*ready) {
            my_logger("Data ready to read");

            if (auto data = sensor.read_measurement()) {
                std::ostringstream oss;
                oss << "CO2: " << data->co2_ppm << " ppm, Temp: " 
                    << std::fixed << std::setprecision(1) << data->temperature_celsius 
                    << " C, Hum: " << data->humidity_percent_rh << " %RH";
                my_logger(oss.str());
                std::this_thread::sleep_for(std::chrono::milliseconds(5000));
            } else {
                std::ostringstream oss;
                oss << "Failed to read measurement: " << SCD4XerrorCodeToString(data.error());
                my_logger(oss.str());
            }
        } else {
            my_logger("Data not ready to read");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    return 0;
}