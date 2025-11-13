#pragma once

#include <cstddef>
#include <cstdint>

#include "delay_interface.hpp"
#include "i2c_interface.hpp"

class PCA9685 {
public:
    static constexpr uint8_t kDefaultI2CAddress = 0x40;
    static constexpr uint32_t kDefaultOscillatorFrequency = 25'000'000;
    static constexpr uint8_t kPrescaleMin = 3;
    static constexpr uint8_t kPrescaleMax = 255;
    static constexpr uint16_t kResolution = 4096;

    PCA9685(I2C_Interface& i2c, Delay_Interface& delay, uint8_t device_address = kDefaultI2CAddress);

    bool initialize(uint8_t external_clock_prescale = 0);
    bool reset();
    bool sleep();
    bool wakeup();
    bool set_external_clock(uint8_t prescale);
    bool set_pwm_frequency(float frequency_hz);
    bool set_output_mode(bool totem_pole);
    uint16_t get_pwm(uint8_t channel, bool return_off_time = false);
    bool set_pwm(uint8_t channel, uint16_t on_tick, uint16_t off_tick);
    bool set_pin(uint8_t channel, uint16_t value, bool invert = false);
    bool write_microseconds(uint8_t channel, uint16_t microseconds);
    uint8_t read_prescale();

    void set_oscillator_frequency(uint32_t frequency_hz);
    uint32_t oscillator_frequency() const;

private:
    static constexpr uint8_t MODE1 = 0x00;
    static constexpr uint8_t MODE2 = 0x01;
    static constexpr uint8_t LED0_ON_L = 0x06;
    static constexpr uint8_t LED0_ON_H = 0x07;
    static constexpr uint8_t LED0_OFF_L = 0x08;
    static constexpr uint8_t LED0_OFF_H = 0x09;
    static constexpr uint8_t ALL_LED_ON_L = 0xFA;
    static constexpr uint8_t ALL_LED_ON_H = 0xFB;
    static constexpr uint8_t ALL_LED_OFF_L = 0xFC;
    static constexpr uint8_t ALL_LED_OFF_H = 0xFD;
    static constexpr uint8_t PRESCALE = 0xFE;

    static constexpr uint8_t MODE1_RESTART = 0x80;
    static constexpr uint8_t MODE1_EXTCLK = 0x40;
    static constexpr uint8_t MODE1_AI = 0x20;
    static constexpr uint8_t MODE1_SLEEP = 0x10;

    static constexpr uint8_t MODE2_OUTDRV = 0x04;

    static constexpr uint16_t FULL_ON = kResolution;
    static constexpr uint16_t FULL_OFF = kResolution;

    bool write_register(uint8_t reg, uint8_t value);
    bool write_registers(uint8_t reg, const uint8_t* data, size_t length);
    bool read_register(uint8_t reg, uint8_t& value);
    bool read_registers(uint8_t reg, uint8_t* buffer, size_t length);

    I2C_Interface& _i2c;
    Delay_Interface& _delay;
    uint8_t _address;
    uint32_t _oscillator_frequency;
};

