#pragma once

#include <coco/debug.hpp>
#include <coco/platform/Loop_native.hpp>
#include <coco/platform/Uart_native.hpp>


using namespace coco;



/// @brief Drivers for UartWriteRead-Test
/// Linux/Ubuntu: sudo usermod -aG dialout $USER (then restart)
struct Drivers {
    Loop_native loop;

    using Uart = Uart_native;
    Uart uart1{loop, Uart::Format::DEFAULT, 38400, 20ms};
    Uart::Buffer buffer1{uart1, 128};
    Uart uart2{loop, Uart::Format::DEFAULT, 38400, 20ms};
    Uart::Buffer buffer2{uart2, 128};
};

Drivers drivers;
