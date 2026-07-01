#pragma once

#include <coco/platform/Loop_TIM2.hpp>
#include <coco/platform/Rs485_UART_DMA.hpp>
#include <coco/board/config.hpp>


using namespace coco;


// drivers for Rs485Send-Test
// board: https://www.st.com/en/evaluation-tools/stm32f0discovery.html
struct Drivers {
    Loop_TIM2 loop{APB1_TIMER_CLOCK};
    Rs485_UART_DMA rs485{loop,
        // use USART1
        gpio::PA10 | gpio::AF1, // USART1 RX (PA10)
        gpio::PA9 | gpio::AF1, // USART1 TX (PA9)
        gpio::PA8, // DE (PA8)
        USART1_CLOCK,
        uart::USART1_INFO,
        dma::DMA1_CH3_CH2_INFO,
        //dma::DMA1_CH5_CH4_INFO,

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
    drivers.rs485.UART_IRQHandler();
}
void DMA1_Channel2_3_IRQHandler() {
    drivers.rs485.DMA_Rx_IRQHandler();
}
}
