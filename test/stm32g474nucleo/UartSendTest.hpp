#pragma once

#include <coco/platform/Loop_TIM2.hpp>
#include <coco/platform/Uart_UART_DMA.hpp>
#include <coco/board/config.hpp>


using namespace coco;


/**
 * Drivers for UartTest
 * Board: https://www.st.com/resource/en/user_manual/dm00556337-stm32g4-nucleo-64-boards-mb1367-stmicroelectronics.pdf
 * Connect as follows:
 * CN5 1 -> CN9 3
 */
struct Drivers {
    Loop_TIM2 loop{APB1_TIMER_CLOCK};

    using Uart = Uart_UART_DMA;
    Uart uart{loop,
        gpio::Config::PA10 | gpio::Config::AF7, // USART1 RX (CN9 3)
        gpio::Config::PA9 | gpio::Config::AF7, // USART1 TX (CN5 1)
        gpio::Config::PA12 | gpio::Config::AF7, // USART1 DE (CN10 12), only for testing, DE signal has no function
        USART1_CLOCK,
        usart::USART1_INFO,
        dma::DMA1_CH1_CH2_INFO,
        usart::Config::DEFAULT,

        //gpio::Config::PB8 | gpio::Config::AF7, // USART3 RX (CN5 10)
        //gpio::Config::PB9 | gpio::Config::AF7, // USART3 TX (CN5 9)
        //usart::USART3_INFO,
        //dma::DMA1_CH1_CH2_INFO,
        //usart::Config::DEFAULT,

        38400, // baud rate
        20}; // RX timeout in bit times
    Uart::Buffer<128> sendBuffer{uart};
    Uart::Buffer<128> receiveBuffer{uart};
};

Drivers drivers;

extern "C" {
void USART1_IRQHandler() {
//void USART3_IRQHandler() {
    drivers.uart.UART_IRQHandler();
}
void DMA1_Channel1_IRQHandler() {
    drivers.uart.DMA_Rx_IRQHandler();
}
}
