#pragma once

#include <coco/platform/Loop_TIM2.hpp>
#include <coco/platform/Uart_UART_DMA.hpp>
#include <coco/board/config.hpp>
#include <coco/debug.hpp>


using namespace coco;


/// @brief Drivers for Uart-Test
/// Board: https://www.st.com/en/evaluation-tools/stm32f0discovery.html
/// Connect RX and TX to test the loopback:
/// PA9 -> PA10
struct Drivers {
    Loop_TIM2 loop{APB1_TIMER_CLOCK};

    using Uart = Uart_UART_DMA;
    Uart uart{loop,
        uart::USART1_INFO,
        gpio::PA10 | gpio::AF1 | gpio::Config::PULL_UP, // USART1 RX (PA10)
        gpio::PA9 | gpio::AF1, // USART1 TX (PA9)
        dma::DMA1_CH3_CH2_INFO,
        //dma::DMA1_CH5_CH4_INFO,
        USART1_CLOCK,

        // USART2 has no receiver timeout
        //uart::USART2_INFO,
        //gpio::PA3 | gpio::AF1, // USART2 RX
        //gpio::PA2 | gpio::AF1, // USART2 TX
        //dma::DMA1_CH5_CH4_INFO,
        //USART2_CLOCK,

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
//void USART2_IRQHandler() {
    drivers.uart.UART_IRQHandler();
}
void DMA1_Channel2_3_IRQHandler() {
//void DMA1_Channel4_5_IRQHandler() {
    drivers.uart.DMA_Rx_IRQHandler();
}
}
