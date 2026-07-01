#pragma once

#include <coco/platform/Loop_TIM2.hpp>
#include <coco/platform/Uart_UART_DMA.hpp>
#include <coco/board/config.hpp>


using namespace coco;


/// @brief Drivers for Uart-Test
/// Board: https://www.st.com/en/evaluation-tools/nucleo-g491re.html
/// Connect RX and TX to test the loopback
struct Drivers {
    Loop_TIM2 loop{APB1_TIMER_CLOCK};

    using Uart = Uart_UART_DMA;
    Uart uart{loop,
        // use USART1
        gpio::PA10 | gpio::AF7 | gpio::Config::PULL_UP, // USART1 RX (CN9 3)
        gpio::PA9 | gpio::AF7, // USART1 TX (CN5 1)
        gpio::PA12 | gpio::AF7, // USART1 DE (CN10 12), only for testing, DE signal has no function
        USART1_CLOCK,
        uart::USART1_INFO,
        dma::DMA1_CH1_CH2_INFO,

        // use USART3
        //gpio::PB8 | gpio::AF7, // USART3 RX (CN5 10)
        //gpio::PB9 | gpio::AF7, // USART3 TX (CN5 9)
        //uart::USART3_INFO,
        //dma::DMA1_CH1_CH2_INFO,

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
//void USART3_IRQHandler() {
    drivers.uart.UART_IRQHandler();
}
void DMA1_Channel1_IRQHandler() {
    drivers.uart.DMA_Rx_IRQHandler();
}
}
