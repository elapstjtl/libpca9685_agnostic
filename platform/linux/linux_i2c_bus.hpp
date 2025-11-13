#pragma once
#include "../../include/libpca9685_agnostic/i2c_interface.hpp"
#include <string>

class LinuxI2CBus : public I2C_Interface {
public:
    /**
     * @param device_path Linux I2C device file path, e.g., "/dev/i2c-1"
     */
    explicit LinuxI2CBus(const std::string& device_path);

    // Using RAII pattern: automatically close file and release resources in destructor
    ~LinuxI2CBus() override;

    // Implement pure virtual functions from interface
    bool write(uint8_t device_address, const uint8_t* data, size_t len) override;
    bool read(uint8_t device_address, uint8_t* data, size_t len) override;

    // Check if successfully initialized
    bool is_valid() const { return _fd >= 0; }

private:
    // Delete copy constructor and assignment to prevent incorrect resource copying
    LinuxI2CBus(const LinuxI2CBus&) = delete;
    LinuxI2CBus& operator=(const LinuxI2CBus&) = delete;

    int _fd; // File descriptor for I2C device
    std::string _device_path;
};