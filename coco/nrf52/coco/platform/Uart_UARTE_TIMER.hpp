#pragma once

#include <coco/Uart.hpp>
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
    /**
     * Constructor
     * @param loop event loop
     * @param rxPin receive pin (RX) and configuration (e.g. PULL_UP, can be NONE)
     * @param txPin transmit pin (TX) pin (e.g. DRIVE_H0H1, can be NONE)
     * @param instances instances to use
     * @param ppiChannels two ppi channels
     * @param baudRate baud rate
     * @param format frame format
     * @param rxTimeout receive timeout in bit times
     */
    Uart_UARTE_TIMER(Loop_Queue &loop, gpio::Config rxPin, gpio::Config txPin, const uart::InfoE &uartInfo, const timer::Info &timerInfo,
        ppi::DualChannel ppiChannels, uart::Config config, int baudRate, int rxTimeout);

    ~Uart_UARTE_TIMER() override;

    class BufferBase;

    // Device methods
    //StateTasks<const State, Events> &getStateTasks() override;

    // BufferDevice methods
    int getBufferCount();
    BufferBase &getBuffer(int index);

    // Uart methods
    // Note: Format only supports 8 data bits, no/even parity and 1/2 stop bits
    void setValue(int id, int value) override;
    int getValue(int id) override;


    // internal buffer base class, derives from IntrusiveListNode for the list of buffers and Loop_Queue::Handler to be notified from the event loop
    class BufferBase : public coco::Buffer, public IntrusiveListNode, public Loop_Queue::Handler {
        friend class Uart_UARTE_TIMER;
    public:
        /**
            Constructor
            @param data data of the buffer
            @param capacity capacity of the buffer
            @param channel channel to attach to
        */
        BufferBase(uint8_t *data, int size, Uart_UARTE_TIMER &device);
        ~BufferBase() override;

        // Device methods
        bool start(Op op) override;
        bool cancel() override;

    protected:
        void startRx();
        void startTx();
        void handle() override;

        Uart_UARTE_TIMER &device;

        Op op;
    };

    /**
     * Buffer for transferring data over UART.
     * @tparam C capacity of buffer
     */
    template <int C>
    class Buffer : public BufferBase {
    public:
        Buffer(Uart_UARTE_TIMER &device) : BufferBase(data, C, device) {}

    protected:
        alignas(4) uint8_t data[C];
    };

    /**
     * UART interrupt handler, needs to be called from UART interrupt handler (UARTE0_UART0_IRQHandler() or UARTE1_IRQHandler())
     */
    void UARTE_IRQHandler();

protected:
    Loop_Queue &loop;
    int baudRate;

    // uart
    NRF_UARTE_Type *uart;
    int uartIrq;

    // timer
    NRF_TIMER_Type *timer;
    int ppiFlags;

    // device state
    //StateTasks<State, Events> st = State::READY;

    // list of buffers
    IntrusiveList<BufferBase> buffers;

    // list of active transfers
    InterruptQueue<BufferBase> receiveTransfers;
    InterruptQueue<BufferBase> sendTransfers;
};

} // namespace coco
