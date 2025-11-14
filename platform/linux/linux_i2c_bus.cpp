/**
 * @file linux_i2c_bus.cpp
 * @brief Linux平台I2C总线实现
 * 
 * 实现了Linux平台下的I2C通信功能，使用Linux I2C设备文件接口。
 */

#include "linux_i2c_bus.hpp"

// Include all necessary Linux system headers
#include <unistd.h>      // POSIX操作系统API：open, close, read, write
#include <fcntl.h>       // 文件控制：O_RDWR等标志
#include <sys/ioctl.h>   // 设备控制：I2C_SLAVE等ioctl命令
#include <linux/i2c-dev.h>  // Linux I2C设备接口定义：I2C_SLAVE等
#include <cstdio>        // 标准I/O：perror用于错误信息

LinuxI2CBus::LinuxI2CBus(const std::string& device_path) : _fd(-1), _device_path(device_path) {
    // 以读写模式打开I2C设备文件
    // O_RDWR: 以读写模式打开，因为I2C通信需要双向数据传输
    // 不使用O_NONBLOCK: I2C操作应该是阻塞的，等待操作完成
    _fd = open(_device_path.c_str(), O_RDWR);
    if (_fd < 0) {
        // 打开失败：文件描述符为负数
        // 使用perror打印详细的系统错误信息，帮助调试
        // 常见错误：ENOENT（文件不存在）、EACCES（权限不足）、EBUSY（设备忙）
        perror(("Failed to open I2C bus: " + _device_path).c_str());
    }
}

LinuxI2CBus::~LinuxI2CBus() {
    // RAII模式：自动关闭文件描述符
    // 检查有效性的原因：避免关闭无效的文件描述符（可能导致未定义行为）
    if (is_valid()) {
        close(_fd);
        // 注意：close()失败时通常不需要处理，因为对象正在销毁
    }
}

bool LinuxI2CBus::write(uint8_t device_address, const uint8_t* data, size_t len) {
    // 检查文件描述符有效性（防御性编程）
    if (!is_valid()) return false;

    // 步骤1：设置从设备地址
    // ioctl I2C_SLAVE: 告诉Linux I2C驱动要与哪个设备通信
    // 为什么每次都要设置？因为同一个I2C总线上可能有多个设备
    // 这是Linux I2C驱动的要求，必须在每次操作前设置地址
    if (ioctl(_fd, I2C_SLAVE, device_address) < 0) {
        perror("Failed to set I2C slave address");
        return false;
    }

    // 步骤2：写入数据
    // 使用POSIX write()系统调用：这是Linux I2C设备文件的标准接口
    // 使用::write而非std::write：明确调用全局命名空间的write函数
    // write()是阻塞的：会等待数据写入完成或发生错误
    ssize_t bytes_written = ::write(_fd, data, len);
    
    // 检查写入的字节数是否与请求的一致
    // 如果不一致，可能表示设备无响应、总线错误等
    if (bytes_written != static_cast<ssize_t>(len)) {
        perror("Failed to write to I2C bus");
        return false;
    }

    return true;
}

bool LinuxI2CBus::read(uint8_t device_address, uint8_t* data, size_t len) {
    // 检查文件描述符有效性（防御性编程）
    if (!is_valid()) return false;
    
    // 步骤1：设置从设备地址（与write()相同的原因）
    if (ioctl(_fd, I2C_SLAVE, device_address) < 0) {
        perror("Failed to set I2C slave address");
        return false;
    }

    // 步骤2：读取数据
    // 使用POSIX read()系统调用：这是Linux I2C设备文件的标准接口
    // read()是阻塞的：会等待数据读取完成或发生错误
    // 对于I2C读取，通常需要先写入寄存器地址（由上层调用者处理）
    ssize_t bytes_read = ::read(_fd, data, len);
    
    // 检查读取的字节数是否与请求的一致
    // 如果不一致，可能表示设备无响应、总线错误等
    if (bytes_read != static_cast<ssize_t>(len)) {
        perror("Failed to read from I2C bus");
        return false;
    }

    return true;
}

bool LinuxI2CBus::write_then_read(uint8_t device_address,
                                   const uint8_t* write_data, size_t write_len,
                                   uint8_t* read_data, size_t read_len) {
    // 检查文件描述符有效性（防御性编程）
    if (!is_valid()) return false;
    
    // 步骤1：设置从设备地址
    // 在同一个函数调用中完成，确保写入和读取使用同一个设备地址
    if (ioctl(_fd, I2C_SLAVE, device_address) < 0) {
        perror("Failed to set I2C slave address");
        return false;
    }

    // 步骤2：写入数据（通常是寄存器地址）
    // 使用POSIX write()系统调用
    ssize_t bytes_written = ::write(_fd, write_data, write_len);
    if (bytes_written != static_cast<ssize_t>(write_len)) {
        perror("Failed to write register address in write_then_read");
        return false;
    }

    // 步骤3：立即读取数据（在同一I2C事务中）
    // 注意：Linux I2C设备文件接口会自动处理重复起始条件（Repeated Start）
    // 这是I2C协议的标准操作：写入地址后，发送重复起始条件，然后读取数据
    // 在同一个函数调用中完成，避免中间被其他线程/设备打断
    ssize_t bytes_read = ::read(_fd, read_data, read_len);
    if (bytes_read != static_cast<ssize_t>(read_len)) {
        perror("Failed to read data in write_then_read");
        return false;
    }

    return true;
}