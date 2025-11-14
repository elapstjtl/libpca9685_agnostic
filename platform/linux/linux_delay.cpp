/**
 * @file linux_delay.cpp
 * @brief Linux平台延迟实现
 * 
 * 实现了Linux平台下的微秒级延迟功能。
 */

#include "linux_delay.hpp"
#include <thread>    // std::this_thread
#include <chrono>    // std::chrono::microseconds

void LinuxDelay::sleep_microseconds(uint32_t usec) {
    // 使用C++11标准库的线程休眠功能
    // std::this_thread::sleep_for(): 暂停当前线程指定的时间
    // std::chrono::microseconds: 表示微秒数的时间类型
    // 
    // 实现细节：
    // - 在Linux上，底层使用nanosleep()系统调用
    // - 精度取决于内核调度器，通常为毫秒级
    // - 对于很小的延迟（<1ms），实际延迟可能远大于请求值
    // 
    // 为什么选择这种方式？
    // 1. 符合C++标准，代码可移植
    // 2. 类型安全，使用std::chrono避免单位错误
    // 3. 自动处理平台差异
    // 
    // 精度限制：
    // - Linux用户空间延迟精度通常为1-10ms
    // - 对于需要精确微秒级延迟的应用，可能需要实时内核或硬件定时器
    std::this_thread::sleep_for(std::chrono::microseconds(usec));
}
