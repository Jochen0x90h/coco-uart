#pragma once

#include <coco/debug.hpp>
#include <coco/platform/Loop_native.hpp>
#include <coco/platform/UartMonitor_native.hpp>
#include <coco/platform/Uart_native.hpp>


using namespace coco;



/// @brief Drivers for Uart-Test
/// Linux/Ubuntu: sudo usermod -aG dialout $USER (then restart)
struct Drivers {
    Loop_native loop;

    UartMonitor_native monitor{loop};

    using Uart = Uart_native;
    Uart uart{loop};
    Uart::Buffer sendBuffer{uart, 128};
    Uart::Buffer receiveBuffer{uart, 128};
};

Drivers drivers;
