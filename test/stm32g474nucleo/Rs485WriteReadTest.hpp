#pragma once

#include <coco/platform/Loop_TIM2.hpp>
#include <coco/platform/Rs485_UART_DMA.hpp>
#include <coco/platform/Uart_UART_DMA.hpp>
#include <coco/board/config.hpp>


using namespace coco;


/**
 * Drivers for Rs485WriteReadTest
 * Board: https://www.st.com/resource/en/user_manual/dm00556337-stm32g4-nucleo-64-boards-mb1367-stmicroelectronics.pdf
 * Connect as follows:
 * CN5 1 -> CN5 10
 * CN5 9 -> CN9 3
 */
struct Drivers {
    Loop_TIM2 loop{APB1_TIMER_CLOCK};

    using Rs485 = Rs485_UART_DMA;
    using Uart = Uart_UART_DMA;
    Rs485 uart1{loop,
        gpio::Config::PA10 | gpio::Config::AF7, // USART1 RX (CN9 3)
        gpio::Config::PA9 | gpio::Config::AF7, // USART1 TX (CN5 1)
        gpio::Config::PA12, // DE (CN10 12)
        USART1_CLOCK,
        usart::USART1_INFO,
        dma::DMA1_CH1_CH2_INFO,
        usart::Config::DEFAULT,
        38400, // baud rate
        20}; // RX timeout in bit times
    Rs485::Buffer<128> buffer1{uart1};

    Rs485 uart2{loop,
        gpio::Config::PB8 | gpio::Config::AF7, // USART3 RX (CN5 10)
        gpio::Config::PB9 | gpio::Config::AF7, // USART3 TX (CN5 9)
        gpio::Config::PA8 | gpio::Config::INVERT, // nDE (CN9 8)
        USART3_CLOCK,
        usart::USART3_INFO,
        dma::DMA1_CH3_CH4_INFO,
        usart::Config::DEFAULT,
        38400, // baud rate
        20}; // RX timeout in bit times
    Rs485::Buffer<128> buffer2{uart2};
};

Drivers drivers;

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
