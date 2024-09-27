#pragma once

#include "Uart_UART_DMA.hpp"


namespace coco {

/**
    Implementation of RS485 interface on STM32 using USARTx or UARTx (LPUARTx is lacking receiver timeout).
    The receiver gets disabled during send and the DE pin can be assigned to any GPIO.

    Resources:
        USART or UART
        DMA
        GPIO for DE pin
*/
class Rs485_UART_DMA : public Uart_UART_DMA {
protected:
    // helper used to configure the DE pin prior to calling the constructor
    static uint32_t configDe(gpio::Config dePin) {
        gpio::configureOutput(dePin, false);
        return 0;
    }

public:
    /**
     * Constructor
     * @param loop event loop
     * @param rxPin receive pin (RX), alternative function (see data sheet) and configuration (e.g. PULL_UP, INVERT, can be NONE)
     * @param txPin transmit pin (TX), alternative function (see data sheet) and configuration (e.g. INVERT, can be NONE)
     * @param dePin data enable pin (DE) and configuration (e.g. INVERT), can be any GPIO
     * @param clock peripheral clock frequency (e.g. APB1_CLOCK)
     * @param uartInfo info of USART/UART instance to use
     * @param dmaInfo info of DMA channels to use
     * @param config configuration, see usart::Config
     * @param baudRate baud rate (e.g. 38400)
     * @param rxTimeout receiver timeout in bit times
     */
    Rs485_UART_DMA(Loop_Queue &loop, gpio::Config rxPin, gpio::Config txPin, gpio::Config dePin,
        Hertz<> clock, const usart::Info &uartInfo, const dma::Info2 &dmaInfo,
        usart::Config config, int baudRate, int rxTimeout)
        : Uart_UART_DMA(loop, rxPin, txPin, clock, uartInfo, dmaInfo, config, baudRate, rxTimeout, configDe(dePin))
        , dePin(dePin)
    {}

    ~Rs485_UART_DMA() override;

protected:
    // Uart_UART_DMA methods
    void startRx(BufferBase &buffer) override;
    void startTx(BufferBase &buffer) override;
    void endTx() override;

    // pins
    gpio::Config dePin;
    bool deState = true;
};

} // namespace coco
