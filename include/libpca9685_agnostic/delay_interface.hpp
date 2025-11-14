/**
 * @file delay_interface.hpp
 * @brief 延迟功能抽象接口
 * 
 * 本文件定义了延迟功能的抽象接口，用于实现平台无关的时序控制。
 * 
 * ## 设计理念
 * 
 * 为什么需要延迟接口？
 * 1. **精度要求不同**：不同平台提供不同精度的延迟函数
 *    - Linux: std::this_thread::sleep_for()（可能不够精确）
 *    - 嵌入式系统: 可能需要使用硬件定时器实现精确延迟
 * 2. **可测试性**：可以注入模拟对象，控制测试中的时间流逝
 * 3. **可移植性**：核心代码不依赖特定的延迟实现
 * 
 * ## 精度说明
 * 
 * 不同平台的延迟精度可能不同：
 * - Linux用户空间：通常精度为毫秒级，微秒级延迟可能不够精确
 * - 嵌入式RTOS：可能提供更精确的微秒级延迟
 * - 裸机系统：可以使用硬件定时器实现精确延迟
 * 
 * @note 实现类应该尽可能提供精确的延迟，但实际精度取决于平台能力
 * @note 延迟操作是阻塞的，会暂停当前线程/任务
 */

#pragma once
#include <cstdint>

namespace libpca9685_agnostic {

/**
 * @brief 延迟功能抽象接口
 * 
 * 定义了微秒级延迟操作，所有平台相关的延迟实现都必须实现此接口。
 * 
 * ## 使用示例
 * 
 * ```cpp
 * class MyDelay : public Delay_Interface {
 *     void sleep_microseconds(uint32_t usec) override {
 *         // 平台特定的实现
 *     }
 * };
 * ```
 */
class Delay_Interface {
public:
    /**
     * @brief 虚析构函数
     * @note 使用default实现，允许通过基类指针安全删除派生类对象
     */
    virtual ~Delay_Interface() = default;
    
    /**
     * @brief 延迟指定的微秒数
     * @param usec 要延迟的微秒数
     * @note 延迟是阻塞的，会暂停当前线程/任务直到延迟完成
     * @note 实际延迟时间可能略长于请求值，取决于平台精度
     * @note 如果usec=0，函数应该立即返回（不延迟）
     * @note 如果usec值很大，实现类应该处理溢出问题
     */
    virtual void sleep_microseconds(uint32_t usec) = 0;
};

} // namespace libpca9685_agnostic