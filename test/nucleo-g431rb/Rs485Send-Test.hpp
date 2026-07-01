#pragma once

#include <coco/platform/Loop_TIM2.hpp>
#include <coco/platform/Rs485_UART_DMA.hpp>
#include <coco/board/config.hpp>


using namespace coco;


/// @brief Drivers for Rs485Send-Test
/// Board: https://www.st.com/resource/en/user_manual/dm00556337-stm32g4-nucleo-64-boards-mb1367-stmicroelectronics.pdf
struct Drivers {
    Loop_TIM2 loop{APB1_TIMER_CLOCK};
    Rs485_UART_DMA rs485{loop,
        gpio::PA10 | gpio::AF7, // USART1 RX (CN9 3)
        gpio::PA9 | gpio::AF7, // USART1 TX (CN5 1)
        gpio::PA12, // DE (CN10 12)
        USART1_CLOCK,
        uart::USART1_INFO,
        dma::DMA1_CH1_CH2_INFO,

        //gpio::PB(4, 7), // USART2 RX (CN10 27)
        //gpio::PB(3, 7), // USART2 TX (CN10 31)
        //gpio::PA(12, 1), // nDE (CN10 12)
        //uart::USART2_INFO,
        //dma::DMA1_CH1_CH2_INFO,

        //gpio::PB(8, 7), // USART3 RX (CN5 10)
        //gpio::PB(9, 7), // USART3 TX (CN5 9)
        //gpio::PA(12), // DE (CN10 12)
        //uart::USART3_INFO,
        //dma::DMA1_CH1_CH2_INFO,

        uart::Config::DEFAULT,
        uart::Format::DEFAULT,
        38400, // baud rate
        20}; // RX timeout in bit times
    Rs485_UART_DMA::Buffer<128> sendBuffer{rs485};
    Rs485_UART_DMA::Buffer<128> receiveBuffer{rs485};
};

Drivers drivers;

// Interrupt handlers (check in startup code if the handler name exists to prevent typos)
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
