#include "linux_delay.hpp"
#include <thread>
#include <chrono>

void LinuxDelay::sleep_microseconds(uint32_t usec) {
    std::this_thread::sleep_for(std::chrono::microseconds(usec));
}
