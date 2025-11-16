/**
 * @file pca9685.hpp
 * @brief PCA9685 16通道PWM控制器驱动类
 * 
 * 本文件定义了PCA9685芯片的C++驱动接口。PCA9685是NXP公司生产的16通道、12位分辨率PWM控制器，
 * 通过I2C总线通信，广泛应用于舵机控制、LED调光等场景。
 * 
 * ## 设计理念
 * 
 * 本驱动采用依赖注入（Dependency Injection）模式，通过抽象接口实现平台无关性：
 * - I2C_Interface: 抽象I2C通信接口，允许在不同平台（Linux、Arduino、ESP32等）上使用
 * - Delay_Interface: 抽象延迟接口，允许使用不同精度的延迟实现
 * 
 * 这种设计使得核心驱动代码可以在任何支持I2C的平台上复用，只需提供对应的平台实现即可。
 * 
 * ## 关键设计决策
 * 
 * 1. **为什么使用接口而非直接调用平台API？**
 *    - 提高可测试性：可以注入模拟对象进行单元测试
 *    - 提高可移植性：核心代码无需修改即可移植到新平台
 *    - 符合SOLID原则中的依赖倒置原则（DIP）
 * 
 * 2. **为什么PWM分辨率是4096？**
 *    - PCA9685内部使用12位计数器（2^12 = 4096）
 *    - 每个PWM周期被分为4096个时间片（tick）
 *    - 更高的分辨率意味着更平滑的PWM输出和更精确的控制
 * 
 * 3. **为什么默认振荡器频率是25MHz？**
 *    - 这是PCA9685芯片的典型内部振荡器频率
 *    - 实际频率可能因芯片批次而异（通常24-26MHz），可通过set_oscillator_frequency()校准
 *    - 频率精度直接影响PWM频率和脉宽的计算准确性
 * 
 * 4. **为什么Prescale范围是3-255？**
 *    - 根据PCA9685数据手册，prescale值必须≥3才能保证PWM输出稳定
 *    - 最大值255是8位寄存器的上限
 *    - Prescale用于分频：实际PWM频率 = 振荡器频率 / (4096 * (prescale + 1))
 * 
 * ## 使用示例
 * 
 * ```cpp
 * // 创建平台相关的I2C和延迟实现
 * LinuxI2CBus i2c("/dev/i2c-1");
 * LinuxDelay delay;
 * 
 * // 创建PCA9685驱动实例
 * PCA9685 pwm(i2c, delay);
 * 
 * // 初始化并设置PWM频率（50Hz适用于舵机）
 * if (pwm.initialize() && pwm.set_pwm_frequency(50.0F)) {
 *     // 设置通道0的PWM占空比为50%
 *     pwm.set_pin(0, 2048);
 * }
 * ```
 * 
 * @note 本类不是线程安全的。如果需要在多线程环境中使用，需要外部同步机制。
 * @note 所有I2C操作都是阻塞的，操作失败时返回false，不会抛出异常。
 * @note 修改prescale寄存器时必须先进入睡眠模式，这是PCA9685芯片的硬件要求。
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "delay_interface.hpp"
#include "i2c_interface.hpp"

class PCA9685 {
public:
    /// @brief PCA9685的默认I2C地址（7位地址，0x40 = 0b01000000）
    /// @note 实际I2C地址可通过硬件地址引脚配置为0x40-0x7F
    static constexpr uint8_t kDefaultI2CAddress = 0x40;
    
    /// @brief 默认振荡器频率（25MHz）
    /// @note 实际频率可能因芯片而异，建议使用示波器校准后调用set_oscillator_frequency()
    static constexpr uint32_t kDefaultOscillatorFrequency = 25'000'000;
    
    /// @brief Prescale寄存器的最小值
    /// @note 根据数据手册，prescale必须≥3才能保证PWM输出稳定
    static constexpr uint8_t kPrescaleMin = 3;
    
    /// @brief Prescale寄存器的最大值（8位寄存器上限）
    static constexpr uint8_t kPrescaleMax = 255;
    
    /// @brief PWM分辨率（12位 = 4096个tick）
    /// @note 每个PWM周期被分为4096个时间片，提供12位精度
    static constexpr uint16_t kResolution = 4096;

    /**
     * @brief 构造函数（自动初始化）
     * @param i2c I2C通信接口的引用（必须保持有效直到对象销毁）
     * @param delay 延迟接口的引用（必须保持有效直到对象销毁）
     * @param device_address I2C设备地址，默认为0x40
     * @param auto_init 是否在构造时自动初始化，默认为true
     * @param external_clock_prescale 外部时钟prescale值（仅在auto_init=true时有效），0表示使用内部振荡器
     * @note 使用引用而非指针，确保接口对象必须存在，避免空指针问题
     * @note 接口对象的生命周期必须长于PCA9685对象
     * @note 如果auto_init=true，构造函数会自动调用initialize()初始化芯片
     * @note 如果初始化失败，对象仍然可以创建，但is_initialized()会返回false
     * 
     * @example
     * ```cpp
     * // 自动初始化（推荐）
     * PCA9685 pwm(i2c, delay);
     * if (pwm.is_initialized()) {
     *     pwm.set_speed(0, 50.0F);
     * }
     * 
     * // 延迟初始化
     * PCA9685 pwm(i2c, delay, 0x40, false);
     * if (pwm.initialize()) {
     *     pwm.set_speed(0, 50.0F);
     * }
     * ```
     */
    PCA9685(libpca9685_agnostic::I2C_Interface& i2c, libpca9685_agnostic::Delay_Interface& delay, 
            uint8_t device_address = kDefaultI2CAddress, bool auto_init = true, uint8_t external_clock_prescale = 0);

    /**
     * @brief 检查芯片是否已初始化
     * @return 已初始化返回true，未初始化返回false
     * @note 构造函数会自动初始化，但如果初始化失败，此方法会返回false
     */
    bool is_initialized() const;

    /**
     * @brief 初始化PCA9685芯片
     * @param external_clock_prescale 外部时钟prescale值，0表示使用内部振荡器
     * @return 成功返回true，失败返回false
     * @note 如果external_clock_prescale为0，将使用内部振荡器并设置PWM频率为1kHz
     * @note 初始化过程包括：复位芯片、设置时钟源、设置PWM频率
     * @note 可以多次调用此方法重新初始化芯片
     * @pre I2C接口必须已正确初始化
     */
    bool initialize(uint8_t external_clock_prescale = 0);
    
    /**
     * @brief 设置PWM频率
     * @param frequency_hz 目标频率（Hz），范围[1.0, 3500.0]，超出范围会被自动限制
     * @return 成功返回true，失败返回false
     * @note 频率计算公式：frequency = oscillator_freq / (4096 * (prescale + 1))
     * @note 常用频率：50Hz（舵机）、1000Hz（LED调光/电机控制）
     * @note 修改prescale前会自动进入睡眠模式（硬件要求），修改后自动恢复
     */
    bool set_pwm_frequency(float frequency_hz);
    
    /**
     * @brief 设置指定通道的PWM速度/占空比（百分比接口，推荐使用）
     * @param channel 通道号，范围[0, 15]
     * @param speed_percent 速度/占空比百分比（0.0-100.0），0.0=0%，100.0=100%
     * @param invert true表示反转输出（低电平有效），false表示正常输出
     * @return 成功返回true，失败或通道无效返回false
     * @note 这是最简化的接口，自动将百分比转换为PWM值
     * @note 适用于电机速度控制、LED亮度控制等场景
     * @note 超出范围的值会被自动限制在[0.0, 100.0]范围内
     * 
     * @example
     * ```cpp
     * // 设置通道0为50%速度
     * pwm.set_speed(0, 50.0F);
     * 
     * // 设置通道1为100%亮度
     * pwm.set_speed(1, 100.0F);
     * 
     * // 设置通道2为全开（100%）
     * pwm.set_speed(2, 100.0F);
     * 
     * // 设置通道3为全关（0%）
     * pwm.set_speed(3, 0.0F);
     * ```
     */
    bool set_speed(uint8_t channel, float speed_percent, bool invert = false);
    
    /**
     * @brief 设置指定通道的PWM脉宽（微秒单位，适用于舵机控制）
     * @param channel 通道号，范围[0, 15]
     * @param microseconds 脉宽（微秒），通常舵机范围是500-2500微秒
     * @return 成功返回true，失败返回false
     * @note 根据当前prescale和振荡器频率自动计算对应的PWM值
     * @note 脉宽会被限制在有效范围内
     * @pre 必须先调用set_pwm_frequency()设置合适的频率（通常50Hz用于舵机）
     * 
     * @example
     * ```cpp
     * // 设置舵机到中间位置（1500微秒）
     * pwm.write_microseconds(0, 1500);
     * ```
     */
    bool write_microseconds(uint8_t channel, uint16_t microseconds);

private:
    /**
     * @brief 复位PCA9685芯片（内部使用）
     * @return 成功返回true，失败返回false
     * @note 通过设置MODE1寄存器的RESTART位来复位芯片
     * @note initialize()内部会自动调用此函数
     */
    bool reset();
    
    /**
     * @brief 使芯片进入睡眠模式（内部使用）
     * @return 成功返回true，失败返回false
     * @note 睡眠模式下PWM输出停止，但寄存器值保持不变
     * @note set_pwm_frequency()内部会自动调用此函数
     */
    bool sleep();
    
    /**
     * @brief 唤醒芯片（内部使用）
     * @return 成功返回true，失败返回false
     * @note set_pwm_frequency()内部会自动调用此函数
     */
    bool wakeup();
    
    /**
     * @brief 配置使用外部时钟源（内部使用）
     * @param prescale 外部时钟的prescale值，范围[kPrescaleMin, kPrescaleMax]
     * @return 成功返回true，失败返回false
     * @note initialize()内部会自动调用此函数（当external_clock_prescale != 0时）
     */
    bool set_external_clock(uint8_t prescale);
    
    /**
     * @brief 设置输出模式（高级功能，内部使用）
     * @param totem_pole true表示推挽输出，false表示开漏输出
     * @return 成功返回true，失败返回false
     * @note 默认使用推挽输出，普通用户通常不需要修改
     */
    bool set_output_mode(bool totem_pole);
    
    /**
     * @brief 读取指定通道的PWM值（内部使用）
     * @param channel 通道号，范围[0, 15]
     * @param return_off_time true返回OFF时间，false返回ON时间
     * @return PWM值（0-4095），失败或通道无效返回0
     */
    uint16_t get_pwm(uint8_t channel, bool return_off_time = false);
    
    /**
     * @brief 设置指定通道的PWM值（精确控制，内部使用）
     * @param channel 通道号，范围[0, 15]
     * @param on_tick ON时间点（0-4095）
     * @param off_tick OFF时间点（0-4095）
     * @return 成功返回true，失败或通道无效返回false
     * @note set_pin()和set_speed()内部调用此函数
     */
    bool set_pwm(uint8_t channel, uint16_t on_tick, uint16_t off_tick);
    
    /**
     * @brief 设置指定通道的PWM占空比（内部使用）
     * @param channel 通道号，范围[0, 15]
     * @param value 占空比值（0-4095），0=0%，4095≈100%
     * @param invert true表示反转输出，false表示正常输出
     * @return 成功返回true，失败或通道无效返回false
     * @note set_speed()内部调用此函数
     */
    bool set_pin(uint8_t channel, uint16_t value, bool invert = false);
    
    /**
     * @brief 读取当前的prescale值（内部使用）
     * @return prescale值（3-255），失败返回0
     * @note write_microseconds()内部调用此函数
     */
    uint8_t read_prescale();
    
    /**
     * @brief 设置振荡器频率（内部使用，用于校准）
     * @param frequency_hz 振荡器频率（Hz），默认25MHz
     * @note 实际芯片的振荡器频率可能略有偏差，可通过示波器测量后校准
     * @note 频率精度直接影响PWM频率和脉宽的计算准确性
     */
    void set_oscillator_frequency(uint32_t frequency_hz);
    
    /**
     * @brief 获取当前设置的振荡器频率（内部使用）
     * @return 振荡器频率（Hz）
     */
    uint32_t oscillator_frequency() const;

    // PCA9685寄存器地址（根据数据手册定义）
    static constexpr uint8_t MODE1 = 0x00;           ///< 模式寄存器1：控制睡眠、重启、时钟源等
    static constexpr uint8_t MODE2 = 0x01;           ///< 模式寄存器2：控制输出驱动模式
    static constexpr uint8_t LED0_ON_L = 0x06;       ///< LED0的ON时间低字节（起始寄存器）
    static constexpr uint8_t LED0_ON_H = 0x07;       ///< LED0的ON时间高字节
    static constexpr uint8_t LED0_OFF_L = 0x08;      ///< LED0的OFF时间低字节
    static constexpr uint8_t LED0_OFF_H = 0x09;      ///< LED0的OFF时间高字节
    static constexpr uint8_t ALL_LED_ON_L = 0xFA;   ///< 所有LED的ON时间低字节（广播寄存器）
    static constexpr uint8_t ALL_LED_ON_H = 0xFB;    ///< 所有LED的ON时间高字节
    static constexpr uint8_t ALL_LED_OFF_L = 0xFC;   ///< 所有LED的OFF时间低字节
    static constexpr uint8_t ALL_LED_OFF_H = 0xFD;   ///< 所有LED的OFF时间高字节
    static constexpr uint8_t PRESCALE = 0xFE;        ///< Prescale寄存器：控制PWM频率

    // MODE1寄存器位定义
    static constexpr uint8_t MODE1_RESTART = 0x80;  ///< 重启位：置1后自动清零，用于软件复位
    static constexpr uint8_t MODE1_EXTCLK = 0x40;    ///< 外部时钟使能位：1=外部时钟，0=内部时钟
    static constexpr uint8_t MODE1_AI = 0x20;        ///< 自动递增位：1=写入后地址自动递增
    static constexpr uint8_t MODE1_SLEEP = 0x10;    ///< 睡眠位：1=睡眠模式，0=正常工作

    // MODE2寄存器位定义
    static constexpr uint8_t MODE2_OUTDRV = 0x04;    ///< 输出驱动模式：1=推挽，0=开漏

    // 特殊PWM值（用于全开/全关）
    static constexpr uint16_t FULL_ON = kResolution;  ///< 全开值：当OFF=0且ON≥4096时，输出始终为高
    static constexpr uint16_t FULL_OFF = kResolution; ///< 全关值：当ON=0且OFF≥4096时，输出始终为低

    /**
     * @brief 写入单个寄存器
     * @param reg 寄存器地址
     * @param value 要写入的值
     * @return 成功返回true，失败返回false
     * @note I2C写入格式：先发送寄存器地址，再发送数据
     */
    bool write_register(uint8_t reg, uint8_t value);
    
    /**
     * @brief 写入多个连续寄存器
     * @param reg 起始寄存器地址
     * @param data 要写入的数据缓冲区
     * @param length 数据长度（最大4字节，受缓冲区大小限制）
     * @return 成功返回true，失败返回false
     * @note 使用I2C的连续写入模式，提高效率
     * @note 缓冲区大小限制为5字节（1字节寄存器地址 + 4字节数据）
     */
    bool write_registers(uint8_t reg, const uint8_t* data, size_t length);
    
    /**
     * @brief 读取单个寄存器
     * @param reg 寄存器地址
     * @param value 用于存储读取值的引用
     * @return 成功返回true，失败返回false
     * @note 使用原子操作write_then_read：先写入寄存器地址，然后立即读取数据
     * @note 确保写入和读取在同一个I2C事务中完成，避免被其他设备打断
     * @note 符合I2C协议规范（使用重复起始条件Repeated Start）
     */
    bool read_register(uint8_t reg, uint8_t& value);
    
    /**
     * @brief 读取多个连续寄存器
     * @param reg 起始寄存器地址
     * @param buffer 用于存储读取数据的缓冲区
     * @param length 要读取的字节数
     * @return 成功返回true，失败返回false
     * @note 使用原子操作write_then_read：先写入寄存器地址，然后立即读取多个字节
     * @note 确保写入和读取在同一个I2C事务中完成，避免被其他设备打断
     * @note PCA9685支持自动递增（AI位），可以连续读取多个寄存器
     * @note 符合I2C协议规范（使用重复起始条件Repeated Start）
     */
    bool read_registers(uint8_t reg, uint8_t* buffer, size_t length);

    libpca9685_agnostic::I2C_Interface& _i2c;  ///< I2C通信接口引用
    libpca9685_agnostic::Delay_Interface& _delay;  ///< 延迟接口引用
    uint8_t _address;  ///< I2C设备地址
    uint32_t _oscillator_frequency;  ///< 振荡器频率（用于PWM计算）
    bool _initialized;  ///< 初始化状态标志
};

