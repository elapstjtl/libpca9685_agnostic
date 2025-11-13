#pragma once
#include <cstdint>
#include <cstddef>

/**
 * @brief define an abstract interface for I2C bus capability
 */
class I2C_Interface {
public:
    virtual ~I2C_Interface() = default;
    virtual bool write(uint8_t device_address, const uint8_t* data, size_t len) = 0;
    virtual bool read(uint8_t device_address, uint8_t* data, size_t len) = 0;
};