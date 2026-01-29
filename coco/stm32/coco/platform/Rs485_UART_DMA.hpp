#pragma once

#include "Uart_UART_DMA.hpp"


namespace coco {

/// @brief Implementation of RS485 interface on STM32 using USARTx or UARTx and GPIO for the DE pin.
/// Note: LPUARTx is lacking receiver timeout.
/// The receiver gets disabled during send and the DE pin can be assigned to any GPIO.
///
/// Resources:
///   USART or UART
///   DMA
///   GPIO for DE pin
class Rs485_UART_DMA : public Uart_UART_DMA {
protected:
    // helper used to configure the DE pin prior to calling the constructor
    static uart::Config configDe(uart::Config config, gpio::Config dePin) {
        gpio::enableOutput(dePin, false);
        return config;
    }

public:
    /// Constructor
    /// @param loop Event loop
    /// @param rxPin Receive pin (RX), alternative function (see data sheet) and configuration (e.g. PULL_UP, INVERT, can be NONE)
    /// @param txPin Rransmit pin (TX), alternative function (see data sheet) and configuration (e.g. INVERT, can be NONE)
    /// @param dePin Data enable pin (DE) and configuration (e.g. INVERT), can be any GPIO
    /// @param clock Peripheral clock frequency (e.g. APB1_CLOCK)
    /// @param uartInfo Info of USART/UART instance to use
    /// @param dmaInfo Info of DMA channels to use
    /// @param config Configuration, see usart::Config. Typically usart::Config::DEFAULT will do the job.
    /// @param format Format, see usart::Format. Typically usart::Format::DEFAULT will do the job.
    /// @param baudRate Baud rate (e.g. 38400)
    /// @param rxTimeout Receiver timeout in bit times
    Rs485_UART_DMA(Loop_Queue &loop, gpio::Config rxPin, gpio::Config txPin, gpio::Config dePin,
        Hertz<> clock, const UartInfo &uartInfo, const dma::DualInfo<> &dmaInfo,
        uart::Config config, uart::Format format, int baudRate, int rxTimeout)
        : Uart_UART_DMA(loop, rxPin, txPin, clock, uartInfo, dmaInfo, configDe(config, dePin), format, baudRate, rxTimeout)
        , dePin_(dePin)
    {}

    ~Rs485_UART_DMA() override;

protected:
    // Uart_UART_DMA methods
    void startRx(BufferBase &buffer) override;
    void startTx(BufferBase &buffer) override;
    void endTx() override;

    // pins
    gpio::Config dePin_;
    bool deState_ = false;
};

} // namespace coco
