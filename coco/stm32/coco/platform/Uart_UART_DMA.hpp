#pragma once

#include <coco/Uart.hpp>
#include <coco/Frequency.hpp>
#include <coco/InterruptQueue.hpp>
#include <coco/platform/Loop_Queue.hpp>
#include <coco/platform/dma.hpp>
#include <coco/platform/gpio.hpp>
#include <coco/platform/uart.hpp>
#include <coco/platform/nvic.hpp>


namespace coco {

/// @brief Implementation of UART interface on STM32 using USARTx or UARTx (LPUARTx and STM32F4 are lacking receiver timeout).
/// Hardware support for RTS/CTS or DE can be configured, but Uart::setOutputSignals() is not supported.
///
/// Resources:
///   USART/UART supporting Feature::RX_TIMEOUT
///   DMA
///     RX channel (read)
///     TX channel (write)
class Uart_UART_DMA : public Uart {
public:
    // the UART needs to support receiver timeout
    using UartInfo = uart::Info<uart::Feature::BAUD_RATE | uart::Feature::RX_TIMEOUT>;
    using DmaInfo = dma::DualInfo<>;

    /// @brief Constructor with only RX and TX pins.
    /// @param loop Event loop
    /// @param rxPin Receive pin (RX), alternative function (see data sheet) and configuration (e.g. PULL_UP, INVERT, can be NONE)
    /// @param txPin Transmit pin (TX), alternative function (see data sheet) and configuration (e.g. INVERT, can be NONE)
    /// @param clock Peripheral clock frequency (e.g. APB1_CLOCK)
    /// @param uartInfo Info of USART/UART instance to use
    /// @param dmaInfo Info of DMA channels to use
    /// @param config Configuration, see usart::Config. Typically usart::Config::DEFAULT will do the job.
    /// @param format Format, see usart::Format. Typically usart::Format::DEFAULT will do the job.
    /// @param baudRate Baud rate (e.g. 38400)
    /// @param rxTimeout Receiver timeout in bit times
    Uart_UART_DMA(Loop_Queue &loop, gpio::Config rxPin, gpio::Config txPin,
        Hertz<> clock, const UartInfo &uartInfo, const DmaInfo &dmaInfo,
        uart::Config config, uart::Format format, int baudRate, int rxTimeout);

    /// @brief Constructor with DE pin for transmit enable.
    /// @param loop Event loop
    /// @param rxPin Receive pin (RX), alternative function (see data sheet) and configuration (e.g. PULL_UP, INVERT, can be NONE)
    /// @param txPin Transmit pin (TX), alternative function (see data sheet) and configuration (e.g. INVERT, can be NONE)
    /// @param dePin Driver enable pin (DE), alternative function (see data sheet) and configuration (e.g. INVERT)
    /// @param clock Peripheral clock frequency (e.g. APB1_CLOCK)
    /// @param uartInfo Info of USART/UART instance to use
    /// @param dmaInfo Info of DMA channels to use
    /// @param config Configuration, see usart::Config. Typically usart::Config::DEFAULT will do the job.
    /// @param format Format, see usart::Format. Typically usart::Format::DEFAULT will do the job.
    /// @param baudRate Baud rate (e.g. 38400)
    /// @param rxTimeout Receiver timeout in bit times
    Uart_UART_DMA(Loop_Queue &loop, gpio::Config rxPin, gpio::Config txPin, gpio::Config dePin,
        Hertz<> clock, const UartInfo &uartInfo, const DmaInfo &dmaInfo,
        uart::Config config, uart::Format format, int baudRate, int rxTimeout)
        : Uart_UART_DMA(loop, rxPin, txPin, clock, uartInfo,
        dmaInfo, uartInfo.enableDePin(dePin, config), format, baudRate, rxTimeout)
    {}

    /// @brief Constructor with RTS/CTS pins for flow control.
    /// @param loop Event loop
    /// @param rxPin Receive pin (RX), alternative function (see data sheet) and configuration (e.g. PULL_UP, INVERT, can be NONE)
    /// @param txPin Transmit pin (TX), alternative function (see data sheet) and configuration (e.g. INVERT, can be NONE)
    /// @param rtsPin Ready to send output pin (RTS), alternative function (see data sheet) and configuration (can be NONE)
    /// @param ctsPin Clear to send input pin (CTS), alternative function (see data sheet) and configuration (e.g. PULL_UP, can be NONE)
    /// @param clock Peripheral clock frequency (e.g. APB1_CLOCK)
    /// @param uartInfo Info of USART/UART instance to use
    /// @param dmaInfo Info of DMA channels to use
    /// @param config Configuration, see usart::Config. Typically usart::Config::DEFAULT will do the job.
    /// @param format Format, see usart::Format. Typically usart::Format::DEFAULT will do the job.
    /// @param baudRate Baud rate (e.g. 38400)
    /// @param rxTimeout Receiver timeout in bit times
    Uart_UART_DMA(Loop_Queue &loop, gpio::Config rxPin, gpio::Config txPin, gpio::Config rtsPin, gpio::Config ctsPin,
        Hertz<> clock, const UartInfo &uartInfo, const DmaInfo &dmaInfo,
        uart::Config config, uart::Format format, int baudRate, int rxTimeout)
        : Uart_UART_DMA(loop, rxPin, txPin, clock, uartInfo,
        dmaInfo, uartInfo.enableRtsCtsPins(rtsPin, ctsPin, config), format, baudRate, rxTimeout)
    {}

    ~Uart_UART_DMA() override;

    class BufferBase;

    // BufferDevice methods
    int getBufferCount() override;
    BufferBase &getBuffer(int index) override;

    // Uart methods
    void setValue(int id, int value) override;
    int getValue(int id) override;


    // internal buffer base class, derives from IntrusiveListNode for the list of buffers and Loop_Queue::Handler2 to be notified from the event loop
    class BufferBase : public coco::Buffer, public IntrusiveListNode, public Loop_Queue::Handler {
        friend class Uart_UART_DMA;
    public:
        /// @brief Constructor
        /// @param data data of the buffer
        /// @param capacity capacity of the buffer
        /// @param device Uart device to attach to
        BufferBase(uint8_t *data, int capacity, Uart_UART_DMA &device);
        ~BufferBase() override;

        // Buffer methods
        bool start(Op op) override;
        bool cancel() override;

    protected:
        void handle() override;

        Uart_UART_DMA &device_;
        Op op_;
    };

    /// @brief Buffer for transferring data over UART.
    /// @tparam C capacity of buffer
    template <int C>
    class Buffer : public BufferBase {
    public:
        Buffer(Uart_UART_DMA &device) : BufferBase(data_, C, device) {}

    protected:
        alignas(4) uint8_t data_[C];
    };

    /// @brief UART interrupt handler, needs to be called from USART/UART interrupt handler (e.g. USART1_IRQHandler() for usart::USART1_INFO on STM32G4)
    ///
    void UART_IRQHandler() {
        auto status = this->uart_.status();

        // check if receive timed out
        if ((status & uart::Status::RX_TIMEOUT) != 0)
            handleRx();

        // check if transmission has completed
        if ((status & uart::Status::TX_COMPLETE) != 0)
            handleTx();
    }

    /// @brief Handle Rx DMA interrupt, needs to be called from Rx DMA channel interrupt handler.
    /// First channel of dma::DualInfo, see startup_stm32XXX.c, e.g. DMA1_Channel1_IRQHandler()
    void DMA_Rx_IRQHandler() {
        // check if receive has completed
        if ((this->rxChannel_.status() & dma::Status::TRANSFER_COMPLETE) != 0)
            handleRx();
    }

protected:
    // stat receiving a buffer
    virtual void startRx(BufferBase &buffer);

    // start transmitting a buffer
    virtual void startTx(BufferBase &buffer);

    // gets called when transmit has ended (from UART interrupt or when interrupts are disabled)
    virtual void endTx();

    // abort active receive transfer and disable receiver
    void disableRx();

    // interrupt handlers
    void handleRx();
    void handleTx();

    Loop_Queue &loop_;

    // peripheral clock
    Hertz<> clock_;

    // uart
    UartInfo::Instance uart_;
    int uartIrq_;

    // dma
    using RxChannel = dma::Channel<dma::Mode::RX8>;
    RxChannel rxChannel_;
    int rxDmaIrq_;
    using TxChannel = dma::Channel<dma::Mode::TX8>;
    TxChannel txChannel_;

    // list of buffers
    IntrusiveList<BufferBase> buffers_;

    // list of active transfers
    InterruptQueue<BufferBase> receiveTransfers_;
    InterruptQueue<BufferBase> sendTransfers_;

    // new baud rate value to be applied when no TX transfer is in progress
    int newBaudRate_ = 0;
};

} // namespace coco
