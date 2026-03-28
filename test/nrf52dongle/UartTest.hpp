#pragma once

#include <coco/platform/Loop_RTC0.hpp>
#include <coco/platform/Uart_UARTE_TIMER.hpp>
#include <coco/board/config.hpp>


using namespace coco;


/// @brief Drivers for UartSendTest
/// Board: https://wiki.makerdiary.com/nrf52840-mdk-usb-dongle/hardware/
/// Connect RX and TX to test the loopback
struct Drivers {
    Loop_RTC0 loop;

    using Uart = Uart_UARTE_TIMER;
    Uart uart{loop,
        // use UARTE1
        gpio::P0_2 | gpio::Config::PULL_UP, // RX (P2)
        gpio::P0_3, // TX (P3, lowest pin on right side when USB points towards top)
        uart::UARTE1_INFO,
        timer::TIMER1_INFO,
        ppi::PPI_CH0_CH1,
        uart::Config::DEFAULT,
        uart::Format::DEFAULT,
        38400, // baud rate
        20}; // RX timeout in bit times
    Uart::Buffer<128> sendBuffer{uart};
    Uart::Buffer<128> receiveBuffer{uart};
};

Drivers drivers;

// Interrupt handlers (check in startup code if the handler name exists to prevent typos)
extern "C" {
void UARTE1_IRQHandler() {
    drivers.uart.UARTE_IRQHandler();
}
}
