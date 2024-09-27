#pragma once

#include <coco/platform/Loop_native.hpp>
#include <coco/platform/Uart_native.hpp>


using namespace coco;



// drivers for UartTest
struct Drivers {
    Loop_native loop;

    using Uart = Uart_native;
    Uart uart{loop};
    Uart::Buffer sendBuffer{uart, 128};
    Uart::Buffer receiveBuffer{uart, 128};

    void init(String device) {
        this->uart.open(device, Uart::Format::DEFAULT, 38400, 20ms);
    }
};

Drivers drivers;
