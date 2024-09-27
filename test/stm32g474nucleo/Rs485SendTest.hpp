#pragma once

#include <coco/platform/Loop_TIM2.hpp>
#include <coco/platform/Rs485_UART_DMA.hpp>
#include <coco/board/config.hpp>


using namespace coco;


/**
 * Drivers for Rs485Test
 * Board: https://www.st.com/resource/en/user_manual/dm00556337-stm32g4-nucleo-64-boards-mb1367-stmicroelectronics.pdf
 */
struct Drivers {
    Loop_TIM2 loop{APB1_TIMER_CLOCK};
    Rs485_UART_DMA rs485{loop,
        gpio::Config::PA10 | gpio::Config::AF7, // USART1 RX (CN9 3)
        gpio::Config::PA9 | gpio::Config::AF7, // USART1 TX (CN5 1)
        gpio::Config::PA12, // DE (CN10 12)
        USART1_CLOCK,
        usart::USART1_INFO,
        dma::DMA1_CH1_CH2_INFO,
        usart::Config::DEFAULT,

        //gpio::Config::PB4 | gpio::Config::AF7, // USART2 RX (CN10 27)
        //gpio::Config::PB3 | gpio::Config::AF7, // USART2 TX (CN10 31)
        //gpio::Config::PA12 | gpio::Config::INVERT, // nDE (CN10 12)
        //USART2_CLOCK
        //usart::USART2_INFO,
        //dma::DMA1_CH1_CH2_INFO,
        //usart::Config::DEFAULT,

        //gpio::Config::PB8 | gpio::Config::AF7, // USART3 RX (CN5 10)
        //gpio::Config::PB9 | gpio::Config::AF7, // USART3 TX (CN5 9)
        //gpio::Config::PA12, // DE (CN10 12)
        //USART3_CLOCK
        //usart::USART3_INFO,
        //dma::DMA1_CH1_CH2_INFO,
        //usart::Config::DEFAULT,

        38400, // baud rate
        20}; // RX timeout in bit times
    Rs485_UART_DMA::Buffer<128> sendBuffer{rs485};
    Rs485_UART_DMA::Buffer<128> receiveBuffer{rs485};
};

Drivers drivers;

extern "C" {
void USART1_IRQHandler() {
//void USART2_IRQHandler() {
//void USART3_IRQHandler() {
    drivers.rs485.UART_IRQHandler();
}
void DMA1_Channel1_IRQHandler() {
    drivers.rs485.DMA_Rx_IRQHandler();
}
}
