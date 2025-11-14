/**
 * @file linux_i2c_bus.hpp
 * @brief Linux平台I2C总线实现
 * 
 * 本文件实现了Linux平台下的I2C通信，使用Linux I2C设备文件接口。
 * 
 * ## 实现原理
 * 
 * Linux I2C通信通过设备文件（如/dev/i2c-1）实现：
 * 1. 打开设备文件获取文件描述符
 * 2. 使用ioctl()设置从设备地址
 * 3. 使用read()/write()系统调用进行数据传输
 * 
 * ## 设计决策
 * 
 * 1. **为什么使用RAII模式？**
 *    - 自动管理文件描述符，避免资源泄漏
 *    - 异常安全：即使发生异常，析构函数也会关闭文件
 * 
 * 2. **为什么禁止拷贝？**
 *    - 文件描述符是唯一资源，拷贝会导致多个对象管理同一个文件描述符
 *    - 可能导致重复关闭、竞态条件等问题
 * 
 * 3. **为什么每次读写前都要设置从设备地址？**
 *    - Linux I2C设备文件是共享的，多个设备可能使用同一个总线
 *    - 每次操作前设置地址确保与正确的设备通信
 *    - 这是Linux I2C驱动的要求
 * 
 * ## 权限要求
 * 
 * 使用本类需要：
 * - 对I2C设备文件有读写权限（通常需要root或加入i2c组）
 * - I2C设备文件存在（如/dev/i2c-1）
 * 
 * @note 本实现不是线程安全的，多线程访问需要外部同步
 * @note I2C操作是阻塞的，直到操作完成或失败
 */

#pragma once
#include "libpca9685_agnostic/i2c_interface.hpp"
#include <string>

class LinuxI2CBus : public libpca9685_agnostic::I2C_Interface {
public:
    /**
     * @brief 构造函数：打开I2C设备文件
     * @param device_path Linux I2C设备文件路径，例如"/dev/i2c-1"
     * @note 如果打开失败，文件描述符为-1，可以通过is_valid()检查
     * @note 打开失败时会通过perror()打印错误信息
     * @pre device_path必须指向有效的I2C设备文件
     * @pre 进程必须有权限访问该设备文件
     */
    explicit LinuxI2CBus(const std::string& device_path);

    /**
     * @brief 析构函数：自动关闭I2C设备文件
     * @note 使用RAII模式，自动释放资源
     * @note 如果文件描述符有效，会自动调用close()关闭
     */
    ~LinuxI2CBus() override;

    /**
     * @brief 向I2C设备写入数据
     * @param device_address I2C设备地址（7位地址）
     * @param data 要写入的数据缓冲区
     * @param len 要写入的字节数
     * @return 成功返回true，失败返回false
     * @note 每次写入前都会设置从设备地址（ioctl I2C_SLAVE）
     * @note 使用POSIX write()系统调用进行实际写入
     * @note 写入失败时会通过perror()打印错误信息
     * @pre is_valid()必须返回true
     */
    bool write(uint8_t device_address, const uint8_t* data, size_t len) override;
    
    /**
     * @brief 从I2C设备读取数据
     * @param device_address I2C设备地址（7位地址）
     * @param data 用于存储读取数据的缓冲区
     * @param len 要读取的字节数
     * @return 成功返回true，失败返回false
     * @note 每次读取前都会设置从设备地址（ioctl I2C_SLAVE）
     * @note 使用POSIX read()系统调用进行实际读取
     * @note 读取失败时会通过perror()打印错误信息
     * @note 此方法通常需要先写入寄存器地址，再读取数据（由调用者负责）
     * @pre is_valid()必须返回true
     */
    bool read(uint8_t device_address, uint8_t* data, size_t len) override;
    
    /**
     * @brief 原子操作：先写入数据，然后立即读取数据
     * @param device_address I2C设备地址（7位地址）
     * @param write_data 要写入的数据缓冲区（通常是寄存器地址）
     * @param write_len 要写入的字节数
     * @param read_data 用于存储读取数据的缓冲区
     * @param read_len 要读取的字节数
     * @return 成功返回true，失败返回false
     * @note 在同一个函数调用中完成写入和读取，避免中间被其他线程/设备打断
     * @note 这是I2C读取寄存器的标准做法，符合I2C协议规范
     * @note 使用POSIX write()和read()系统调用，但确保在同一个I2C事务中
     * @pre is_valid()必须返回true
     */
    bool write_then_read(uint8_t device_address,
                         const uint8_t* write_data, size_t write_len,
                         uint8_t* read_data, size_t read_len) override;

    /**
     * @brief 检查I2C设备是否成功打开
     * @return 成功打开返回true，失败返回false
     * @note 通过检查文件描述符是否≥0来判断
     * @note 如果返回false，所有I2C操作都会失败
     */
    bool is_valid() const { return _fd >= 0; }

private:
    /**
     * @brief 禁止拷贝构造函数
     * @note 文件描述符是唯一资源，禁止拷贝避免资源管理问题
     */
    LinuxI2CBus(const LinuxI2CBus&) = delete;
    
    /**
     * @brief 禁止赋值运算符
     * @note 文件描述符是唯一资源，禁止赋值避免资源管理问题
     */
    LinuxI2CBus& operator=(const LinuxI2CBus&) = delete;

    int _fd;  ///< I2C设备文件的文件描述符，-1表示无效
    std::string _device_path;  ///< I2C设备文件路径（用于错误信息）
};