#pragma once

#include <coco/debug.hpp>
#include <coco/platform/Loop_native.hpp>
#include <coco/platform/Uart_native.hpp>


using namespace coco;



/// @brief Drivers for UartSendTest
/// Linux/Ubuntu: sudo usermod -aG dialout $USER (then restart)
struct Drivers {
    Loop_native loop;

    using Uart = Uart_native;
    Uart uart{loop};
    Uart::Buffer sendBuffer{uart, 128};
    Uart::Buffer receiveBuffer{uart, 128};

    void init(String device) {
        if (!uart.open(device, Uart::Format::DEFAULT, 38400, 20ms))
            debug::out << "Error: " << uart.error().message() << '\n';
    }
};

Drivers drivers;
