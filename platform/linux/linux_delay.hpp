#pragma once
#include "../../include/libpca9685_agnostic/delay_interface.hpp"

class LinuxDelay : public Delay_Interface {
public:
    void sleep_microseconds(uint32_t usec) override;
}; 