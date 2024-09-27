#pragma once

#include <coco/platform/Loop_RTC0.hpp>
#include <coco/platform/Uart_UARTE_TIMER.hpp>
#include <coco/board/config.hpp>


using namespace coco;


// drivers for UartTest
// board: https://wiki.makerdiary.com/nrf52840-mdk-usb-dongle/hardware/
struct Drivers {
    Loop_RTC0 loop;

    using Uart = Uart_UARTE_TIMER;
    Uart uart{loop,
        gpio::Config::P0_2, // RX (P2)
        gpio::Config::P0_3, // TX (P3, lowest pin on right side when USB points towards top)
        uart::UARTE0_INFO,
        timer::TIMER1_INFO,
        ppi::PPI_CH0_CH1,
        uart::Config::DEFAULT,
        38400, // baud rate
        20}; // RX timeout in bit times
    Uart::Buffer<128> sendBuffer{uart};
    Uart::Buffer<128> receiveBuffer{uart};
};

Drivers drivers;

extern "C" {
void UARTE0_UART0_IRQHandler() {
    drivers.uart.UARTE_IRQHandler();
}
}
