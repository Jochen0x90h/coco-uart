#pragma once

#include <coco/platform/Loop_TIM2.hpp>
#include <coco/platform/Uart_UART_DMA.hpp>
#include <coco/board/config.hpp>
#include <coco/debug.hpp>


using namespace coco;


/**
 * Drivers for UartTest
 * Board: https://www.st.com/en/evaluation-tools/32f3348discovery.html
 * Connect as follows:
 * PA9 -> PA10
 */
struct Drivers {
    Loop_TIM2 loop{APB1_TIMER_CLOCK};

    using Uart = Uart_UART_DMA;
    Uart uart{loop,
        gpio::Config::PA10 | gpio::Config::AF7, // USART1 RX (PA10)
        gpio::Config::PA9 | gpio::Config::AF7, // USART1 TX (PA9)
        USART1_CLOCK,
        usart::USART1_INFO,
        dma::DMA1_CH5_CH4_INFO,
        usart::Config::DEFAULT,
        38400, // baud rate
        20}; // RX timeout in bit times
    Uart::Buffer<128> sendBuffer{uart};
    Uart::Buffer<128> receiveBuffer{uart};
};

Drivers drivers;

extern "C" {
void USART1_IRQHandler() {
    drivers.uart.UART_IRQHandler();
}
void DMA1_Channel5_IRQHandler() {
    drivers.uart.DMA_Rx_IRQHandler();
}
}
