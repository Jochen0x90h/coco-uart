#pragma once

#include <coco/platform/Loop_SysTick.hpp>
#include <coco/platform/Uart_UART_DMA.hpp>
#include <coco/board/config.hpp>


using namespace coco;


/// @brief Drivers for Uart-Test
/// Board: https://www.st.com/resource/en/user_manual/dm00556337-stm32g4-nucleo-64-boards-mb1367-stmicroelectronics.pdf
/// Connect RX and TX to test the loopback
struct Drivers {
    Loop_SysTick loop{AHB_CLOCK, Loop_SysTick::Mode::POLL};

    using Uart = Uart_UART_DMA;
    Uart uart{loop,
        // use USART1
        USART1_CLOCK,
        gpio::PA10 | gpio::AF1 | gpio::Config::PULL_UP, // USART1 RX (CN9 3)
        gpio::PA9 | gpio::AF1, // USART1 TX (CN5 1)
        gpio::PA12 | gpio::AF1, // USART1 DE (CN10 12), only for testing, DE signal has no function
        uart::USART1_INFO,
        dma::DMA1_CH1_CH2_INFO,

        // use USART3
        //gpio::PB9 | gpio::AF4, // USART3 RX (CN5 9)
        //gpio::PB8 | gpio::AF4, // USART3 TX (CN5 10)
        //USART3_CLOCK,
        //uart::USART3_INFO,
        //dma::DMA1_CH1_CH2_INFO,

        uart::Config::OVER_8,
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
