#pragma once

#include <coco/platform/Loop_TIM2.hpp>
#include <coco/platform/Uart_UART_DMA.hpp>
#include <coco/board/config.hpp>
#include <coco/debug.hpp>


using namespace coco;


/// @brief Drivers for UartSend-Test
/// Board: https://www.st.com/en/evaluation-tools/32f3348discovery.html
/// Connect RX and TX to test the loopback:
/// PA9 -> PA10
struct Drivers {
    Loop_TIM2 loop{APB1_TIMER_CLOCK};

    using Uart = Uart_UART_DMA;
    Uart uart{loop,
        // use USART1
        gpio::PA10 | gpio::AF7, // USART1 RX (PA10)
        gpio::PA9 | gpio::AF7, // USART1 TX (PA9)
        USART1_CLOCK,
        uart::USART1_INFO,
        dma::DMA1_CH5_CH4_INFO,

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
void USART1_IRQHandler() {
    drivers.uart.UART_IRQHandler();
}
void DMA1_Channel5_IRQHandler() {
    drivers.uart.DMA_Rx_IRQHandler();
}
}
