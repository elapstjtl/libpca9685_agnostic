/**
 * @file i2c_interface.hpp
 * @brief I2C总线通信抽象接口
 * 
 * 本文件定义了I2C通信的抽象接口，用于实现平台无关的PCA9685驱动。
 * 
 * ## 设计理念
 * 
 * 使用接口抽象而非直接调用平台API的原因：
 * 1. **可移植性**：核心驱动代码可以在任何支持I2C的平台上运行（Linux、Arduino、ESP32等）
 * 2. **可测试性**：可以注入模拟对象进行单元测试，无需真实的硬件
 * 3. **依赖倒置**：符合SOLID原则，高层模块不依赖低层模块，都依赖抽象
 * 
 * ## 接口设计决策
 * 
 * 1. **为什么使用纯虚函数？**
 *    - 强制子类实现所有方法，确保接口完整性
 *    - 允许运行时多态，支持不同的I2C实现
 * 
 * 2. **为什么返回bool而非抛出异常？**
 *    - 嵌入式系统通常不支持异常处理
 *    - 错误处理更简单直接，适合硬件驱动场景
 *    - 调用者可以立即知道操作是否成功
 * 
 * 3. **为什么使用引用而非指针？**
 *    - 在接口定义中使用引用更安全（不能为空）
 *    - 但在实现中，底层API可能使用指针
 * 
 * @note 所有I2C操作都是阻塞的，操作完成或失败后才返回
 * @note 实现类必须保证线程安全（如果需要在多线程环境中使用）
 */

#pragma once
#include <cstdint>
#include <cstddef>

namespace libpca9685_agnostic {

/**
 * @brief I2C总线通信抽象接口
 * 
 * 定义了I2C总线的基本读写操作，所有平台相关的I2C实现都必须实现此接口。
 * 
 * ## 使用示例
 * 
 * ```cpp
 * class MyI2C : public I2C_Interface {
 *     bool write(uint8_t addr, const uint8_t* data, size_t len) override {
 *         // 平台特定的实现
 *     }
 *     bool read(uint8_t addr, uint8_t* data, size_t len) override {
 *         // 平台特定的实现
 *     }
 * };
 * ```
 */
class I2C_Interface {
public:
    /**
     * @brief 虚析构函数
     * @note 使用default实现，允许通过基类指针安全删除派生类对象
     */
    virtual ~I2C_Interface() = default;
    
    /**
     * @brief 向I2C设备写入数据
     * @param device_address I2C设备地址（7位地址，不包含R/W位）
     * @param data 要写入的数据缓冲区
     * @param len 要写入的字节数
     * @return 成功返回true，失败返回false
     * @note 实现类应该处理I2C总线错误（如设备无响应、总线忙等）
     * @note data指针必须有效且至少包含len字节
     * @pre I2C总线必须已初始化
     * @pre device_address必须是有效的7位I2C地址（0x08-0x77）
     */
    virtual bool write(uint8_t device_address, const uint8_t* data, size_t len) = 0;
    
    /**
     * @brief 从I2C设备读取数据
     * @param device_address I2C设备地址（7位地址，不包含R/W位）
     * @param data 用于存储读取数据的缓冲区
     * @param len 要读取的字节数
     * @return 成功返回true，失败返回false
     * @note 实现类应该处理I2C总线错误（如设备无响应、总线忙等）
     * @note data指针必须有效且至少能容纳len字节
     * @note 此方法通常需要先写入寄存器地址，再读取数据（由调用者负责）
     * @pre I2C总线必须已初始化
     * @pre device_address必须是有效的7位I2C地址（0x08-0x77）
     */
    virtual bool read(uint8_t device_address, uint8_t* data, size_t len) = 0;
    
    /**
     * @brief 原子操作：先写入数据，然后立即读取数据（write-then-read）
     * @param device_address I2C设备地址（7位地址，不包含R/W位）
     * @param write_data 要写入的数据缓冲区（通常是寄存器地址）
     * @param write_len 要写入的字节数
     * @param read_data 用于存储读取数据的缓冲区
     * @param read_len 要读取的字节数
     * @return 成功返回true，失败返回false
     * @note 这是一个原子操作，确保写入和读取在同一个I2C事务中完成
     * @note 避免了两次独立操作之间可能被其他设备打断的问题
     * @note 这是I2C读取寄存器的标准做法，符合I2C协议规范
     * @note write_data和read_data可以是同一个缓冲区（如果大小足够）
     * @pre I2C总线必须已初始化
     * @pre device_address必须是有效的7位I2C地址（0x08-0x77）
     * @pre write_data必须有效且至少包含write_len字节
     * @pre read_data必须有效且至少能容纳read_len字节
     */
    virtual bool write_then_read(uint8_t device_address, 
                                 const uint8_t* write_data, size_t write_len,
                                 uint8_t* read_data, size_t read_len) = 0;
};

} // namespace libpca9685_agnostic