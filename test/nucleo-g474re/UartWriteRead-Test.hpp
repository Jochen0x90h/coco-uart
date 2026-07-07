#pragma once

#include <coco/platform/Loop_TIM2.hpp>
#include <coco/platform/Uart_UART_DMA.hpp>
#include <coco/board/config.hpp>


using namespace coco;


/// @brief Drivers for UartWriteRead-Test
/// Board: https://www.st.com/resource/en/user_manual/dm00556337-stm32g4-nucleo-64-boards-mb1367-stmicroelectronics.pdf
/// Connect as follows:
/// CN5 1 -> CN5 10
/// CN5 9 -> CN9 3
struct Drivers {
    Loop_TIM2 loop{APB1_TIMER_CLOCK};

    using Uart = Uart_UART_DMA;
    Uart uart1{loop,
        uart::USART1_INFO,
        gpio::PA10 | gpio::AF7, // USART1 RX (CN9 3)
        gpio::PA9 | gpio::AF7, // USART1 TX (CN5 1)
        dma::DMA1_CH1_CH2_INFO,
        USART1_CLOCK,
        uart::Config::DEFAULT,
        uart::Format::DEFAULT,
        38400, // baud rate
        20}; // RX timeout in bit times
    Uart::Buffer<128> buffer1{uart1};

    Uart uart2{loop,
        uart::USART3_INFO,
        gpio::PB8 | gpio::AF7, // USART3 RX (CN5 10)
        gpio::PB9 | gpio::AF7, // USART3 TX (CN5 9)
        dma::DMA1_CH3_CH4_INFO,
        USART3_CLOCK,
        uart::Config::DEFAULT,
        uart::Format::DEFAULT,
        38400, // baud rate
        20}; // RX timeout in bit times
    Uart::Buffer<128> buffer2{uart2};
};

Drivers drivers;

// Interrupt handlers (check in startup code if the handler name exists to prevent typos)
extern "C" {
void USART1_IRQHandler() {
    drivers.uart1.UART_IRQHandler();
}
void DMA1_Channel1_IRQHandler() {
    drivers.uart1.DMA_Rx_IRQHandler();
}

void USART3_IRQHandler() {
    drivers.uart2.UART_IRQHandler();
}
void DMA1_Channel3_IRQHandler() {
    drivers.uart2.DMA_Rx_IRQHandler();
}
}
