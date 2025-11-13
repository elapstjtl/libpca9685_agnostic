#include "linux_i2c_bus.hpp"

// Include all necessary Linux system headers
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdio> // For perror

LinuxI2CBus::LinuxI2CBus(const std::string& device_path) : _fd(-1), _device_path(device_path) {
    // O_RDWR means open file in read-write mode
    _fd = open(_device_path.c_str(), O_RDWR);
    if (_fd < 0) {
        // perror will print more detailed system error information, e.g., "No such file or directory"
        perror(("Failed to open I2C bus: " + _device_path).c_str());
    }
}

LinuxI2CBus::~LinuxI2CBus() {
    if (is_valid()) {
        close(_fd);
    }
}

bool LinuxI2CBus::write(uint8_t device_address, const uint8_t* data, size_t len) {
    if (!is_valid()) return false;

    // Use ioctl system call to set the slave device address for communication
    if (ioctl(_fd, I2C_SLAVE, device_address) < 0) {
        perror("Failed to set I2C slave address");
        return false;
    }

    // Use POSIX write system call to send data
    ssize_t bytes_written = ::write(_fd, data, len);
    if (bytes_written != static_cast<ssize_t>(len)) {
        perror("Failed to write to I2C bus");
        return false;
    }

    return true;
}

bool LinuxI2CBus::read(uint8_t device_address, uint8_t* data, size_t len) {
    if (!is_valid()) return false;
    
    if (ioctl(_fd, I2C_SLAVE, device_address) < 0) {
        perror("Failed to set I2C slave address");
        return false;
    }

    // Use POSIX read system call to read data
    ssize_t bytes_read = ::read(_fd, data, len);
    if (bytes_read != static_cast<ssize_t>(len)) {
        perror("Failed to read from I2C bus");
        return false;
    }

    return true;
}