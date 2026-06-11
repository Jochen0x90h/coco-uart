#pragma once

#include <coco/Uart.hpp>
#include <coco/InterruptQueue.hpp>
#include <coco/platform/Loop_Queue.hpp>
#include <coco/platform/gpio.hpp>
#include <coco/platform/nvic.hpp>
#include <coco/platform/ppi.hpp>
#include <coco/platform/timer.hpp>
#include <coco/platform/uart.hpp>


namespace coco {

/**
    Implementation of Uart interface on nrf52 using UARTEx.

    Reference manual:
        https://infocenter.nordicsemi.com/topic/ps_nrf52840/uarte.html?cp=5_0_0_5_33
    Resources:
        UARTE
        TIMER
        PPI two channels
*/
class Uart_UARTE_TIMER : public Uart {
public:
    using UartInfo = uart::Info<uart::Feature::DMA>;

    /// @brief Constructor with only RX and TX pins.
    /// @param loop event loop
    /// @param rxPin receive pin (RX) and configuration (e.g. PULL_UP, can be NONE)
    /// @param txPin transmit pin (TX) pin (e.g. DRIVE_H0H1, can be NONE)
    /// @param instances instances to use
    /// @param ppiChannels two ppi channels
    /// @param baudRate baud rate
    /// @param config Configuration, see usart::Config. Typically usart::Config::DEFAULT will do the job.
    /// @param format Format, see usart::Format. Typically usart::Format::DEFAULT will do the job.
    /// @param rxTimeout receive timeout in bit times
    Uart_UARTE_TIMER(Loop_Queue &loop, gpio::Config rxPin, gpio::Config txPin, const UartInfo &uartInfo, const timer::Info &timerInfo,
        ppi::DualChannel ppiChannels, uart::Config config, uart::Format format, int baudRate, int rxTimeout);

    ~Uart_UARTE_TIMER() override;

    class BufferBase;

    // BufferDevice methods
    int getBufferCount();
    BufferBase &getBuffer(int index);

    // Uart methods
    bool open() override;
    void setValue(int id, int value) override; // Note: Format only supports 8 data bits, no/even parity and 1/2 stop bits
    int getValue(int id) override;


    // internal buffer base class, derives from IntrusiveListNode for the list of buffers and Loop_Queue::Handler to be notified from the event loop
    class BufferBase : public coco::Buffer, public IntrusiveListNode, public Loop_Queue::CompletionHandler {
        friend class Uart_UARTE_TIMER;
    public:
        /// @brief Constructor
        /// @param data data of the buffer
        /// @param capacity capacity of the buffer
        /// @param device Uart device to attach to
        BufferBase(uint8_t *data, int capacity, Uart_UARTE_TIMER &device);
        ~BufferBase() override;

        // Device methods
        bool start() override;
        bool cancel() override;

    protected:
        void startRx();
        void startTx();
        void onCompletion() override;

        Uart_UARTE_TIMER &device_;
        //Op op_;
    };

    /// @param Buffer for transferring data over UART.
    /// @tparam C capacity of buffer
    template <int C>
    class Buffer : public BufferBase {
    public:
        Buffer(Uart_UARTE_TIMER &device) : BufferBase(data_, C, device) {}

    protected:
        alignas(4) uint8_t data_[C];
    };

    /// @brief UART interrupt handler, needs to be called from UART interrupt handler (UARTE0_UART0_IRQHandler() or UARTE1_IRQHandler())
    ///
    void UARTE_IRQHandler();

protected:
    Loop_Queue &loop_;
    int baudRate_;

    // uart
    UartInfo::Instance uart_;
    int uartIrq_;

    // timer
    NRF_TIMER_Type *timer_;
    int ppiFlags_;

    // list of buffers
    IntrusiveList<BufferBase> buffers_;

    // list of active transfers
    InterruptQueue<BufferBase> receiveTransfers_;
    InterruptQueue<BufferBase> sendTransfers_;

    // new baud rate value to be applied when no TX transfer is in progress
    int newBaudRate_ = 0;
};

} // namespace coco
