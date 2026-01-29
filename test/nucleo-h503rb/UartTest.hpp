#pragma once

#include <coco/platform/Loop_TIM2.hpp>
#include <coco/platform/Uart_UART_DMA.hpp>
#include <coco/board/config.hpp>


using namespace coco;


/// @brief Drivers for UartSendTest
/// Board: https://www.st.com/resource/en/user_manual/dm00556337-stm32g4-nucleo-64-boards-mb1367-stmicroelectronics.pdf
/// Connect RX and TX to test the loopback
struct Drivers {
    Loop_TIM2 loop{APB1_TIMER_CLOCK};

    using Uart = Uart_UART_DMA;
    Uart uart{loop,
        // use USART1
        gpio::PB15 | gpio::AF4 | gpio::Config::PULL_UP, // USART1 RX (CN9 1)
        gpio::PB14 | gpio::AF4, // USART1 TX (CN9 2)
        //gpio::PA12 | gpio::AF7, // USART1 DE (CN10 12), only for testing, DE signal has no function
        USART1_CLOCK,
        uart::USART1_INFO,
        dma::DMA1_CH0_CH1_INFO,

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
void GPDMA1_Channel0_IRQHandler() {
    drivers.uart.DMA_Rx_IRQHandler();
}
}
