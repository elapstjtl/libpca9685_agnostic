/**
 * @file linux_delay.hpp
 * @brief Linux平台延迟实现
 * 
 * 本文件实现了Linux平台下的微秒级延迟功能，使用C++标准库的线程休眠。
 * 
 * ## 实现原理
 * 
 * 使用std::this_thread::sleep_for()实现延迟：
 * - 这是C++11标准库提供的跨平台线程休眠接口
 * - 在Linux上，底层使用nanosleep()系统调用
 * 
 * ## 精度说明
 * 
 * **重要限制**：Linux用户空间的延迟精度通常为毫秒级
 * - 实际延迟时间可能略长于请求值（通常多几毫秒）
 * - 对于需要精确微秒级延迟的应用，可能需要：
 *   - 使用实时内核（RT kernel）
 *   - 使用硬件定时器
 *   - 使用忙等待（busy-wait）循环（不推荐，占用CPU）
 * 
 * ## 设计决策
 * 
 * 为什么使用std::this_thread::sleep_for()而非nanosleep()？
 * - 更符合C++标准，代码更可移植
 * - 自动处理平台差异
 * - 类型安全（使用std::chrono类型）
 * 
 * @note 本实现适用于大多数应用场景，但对于需要精确微秒级延迟的场景可能不够精确
 * @note 延迟是阻塞的，会暂停当前线程
 */

#pragma once
#include "libpca9685_agnostic/delay_interface.hpp"

class LinuxDelay : public libpca9685_agnostic::Delay_Interface {
public:
    /**
     * @brief 延迟指定的微秒数
     * @param usec 要延迟的微秒数
     * @note 使用std::this_thread::sleep_for()实现
     * @note 实际延迟精度取决于Linux内核调度器，通常为毫秒级
     * @note 对于很小的延迟值（<1ms），实际延迟可能远大于请求值
     */
    void sleep_microseconds(uint32_t usec) override;
}; 