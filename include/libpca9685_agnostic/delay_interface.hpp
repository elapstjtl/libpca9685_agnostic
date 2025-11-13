#pragma once
#include <cstdint>

/**
 * @brief define an abstract interface for delay capability
 */
class Delay_Interface {
public:
    virtual ~Delay_Interface() = default;
    virtual void sleep_microseconds(uint32_t usec) = 0;
};