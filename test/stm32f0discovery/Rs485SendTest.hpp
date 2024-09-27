#pragma once

#include <coco/platform/Loop_TIM2.hpp>
#include <coco/platform/Rs485_UART_DMA.hpp>
#include <coco/board/config.hpp>


using namespace coco;


// drivers for Rs485Test
// board: https://www.st.com/en/evaluation-tools/stm32f0discovery.html
struct Drivers {
    Loop_TIM2 loop{APB1_TIMER_CLOCK};
    Rs485_UART_DMA rs485{loop,
        gpio::Config::PA10 | gpio::Config::AF1, // USART1 RX (PA10)
        gpio::Config::PA9 | gpio::Config::AF1, // USART1 TX (PA9)
        gpio::Config::PA8, // DE (PA8)
        USART1_CLOCK,
        usart::USART1_INFO,
        dma::DMA1_CH3_CH2_INFO,
        //dma::DMA1_CH5_CH4_INFO,
        usart::Config::DEFAULT,
        38400, // baud rate
        20}; // RX timeout in bit times
    Rs485_UART_DMA::Buffer<128> sendBuffer{rs485};
    Rs485_UART_DMA::Buffer<128> receiveBuffer{rs485};
};

Drivers drivers;

extern "C" {
void USART1_IRQHandler() {
    drivers.rs485.UART_IRQHandler();
}
void DMA1_Channel2_3_IRQHandler() {
    drivers.rs485.DMA_Rx_IRQHandler();
}
}
