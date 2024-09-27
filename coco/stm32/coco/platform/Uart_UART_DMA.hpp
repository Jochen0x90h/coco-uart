#pragma once

#include <coco/Uart.hpp>
#include <coco/Frequency.hpp>
#include <coco/platform/Loop_Queue.hpp>
#include <coco/platform/dma.hpp>
#include <coco/platform/gpio.hpp>
#include <coco/platform/usart.hpp>
#include <coco/platform/nvic.hpp>


namespace coco {

/**
 * Implementation of UART interface on STM32 using USARTx or UARTx (LPUARTx is lacking receiver timeout).
 * Hardware support for RTS/CTS or DE can be configured, but setRts() and setDtr() are not supported.
 *
 * Reference manual:
 *   f0:
 *     https://www.st.com/resource/en/reference_manual/dm00031936-stm32f0x1stm32f0x2stm32f0x8-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
 *       USART: Section 27
 *       DMA: Section 10, Table 29
 *       Code Examples: Section A.19
 *   g4:
 *     https://www.st.com/resource/en/reference_manual/rm0440-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
 *       USART: Section 37
 *       DMA: Section 12
 *       DMAMUX: Section 13
 * Data sheet:
 *   f0:
 *     https://www.st.com/resource/en/datasheet/stm32f042f6.pdf
 *       Alternate Functions: Section 4, Tables 14-16, Page 37
 *     https://www.st.com/resource/en/datasheet/dm00039193.pdf
 *       Alternate Functions: Section 4, Tables 14+15, Page 37
 *   g4:
 *     https://www.st.com/resource/en/datasheet/stm32g431rb.pdf
 *       Alternate Functions: Section 4.11, Table 13, Page 61
 * Resources:
 *   USART or UART
 *   DMA
 */
class Uart_UART_DMA : public Uart {
protected:
    /**
     * Internal constructor
     */
    Uart_UART_DMA(Loop_Queue &loop, gpio::Config rxPinFunction, gpio::Config txPinFunction,
        Hertz<> clock, const usart::Info &uartInfo, const dma::Info2 &dmaInfo,
        usart::Config config, int baudRate, int rxTimeout, uint32_t cr3);

    // helper used to configure DE pin prior to calling the constructor, returns flags for CR3 register
    static uint32_t configDe(gpio::Config dePin) {
        gpio::configureAlternate(dePin);
        return USART_CR3_DEM
            | ((dePin & gpio::Config::INVERT) != 0 ? USART_CR3_DEP : 0); // invert DE
    }

    // helper used to configure RTS/CTS pins prior to calling the constructor, returns flags for CR3 register
    static uint32_t configRtsCts(gpio::Config rtsPin, gpio::Config ctsPin) {
        int cr3 = 0;
        if (rtsPin != gpio::Config::NONE) {
            gpio::configureAlternate(rtsPin);
            cr3 |= USART_CR3_RTSE;
        }
        if (ctsPin != gpio::Config::NONE) {
            gpio::configureAlternate(ctsPin);
            cr3 |= USART_CR3_CTSE;
        }
        return cr3;
    }

public:
    /**
     * Constructor
     * @param loop event loop
     * @param rxPin receive pin (RX), alternative function (see data sheet) and configuration (e.g. PULL_UP, INVERT, can be NONE)
     * @param txPin transmit pin (TX), alternative function (see data sheet) and configuration (e.g. INVERT, can be NONE)
     * @param clock peripheral clock frequency (e.g. APB1_CLOCK)
     * @param uartInfo info of USART/UART instance to use
     * @param dmaInfo info of DMA channels to use
     * @param config configuration, see usart::Config. Typically usart::Config::DEFAULT will do the job.
     * @param baudRate baud rate (e.g. 38400)
     * @param rxTimeout receiver timeout in bit times
     */
    Uart_UART_DMA(Loop_Queue &loop, gpio::Config rxPin, gpio::Config txPin,
        Hertz<> clock, const usart::Info &uartInfo, const dma::Info2 &dmaInfo,
        usart::Config config, int baudRate, int rxTimeout)
        : Uart_UART_DMA(loop, rxPin, txPin, clock, uartInfo, dmaInfo, config, baudRate, rxTimeout, 0) {}

    /**
     * Constructor
     * @param loop event loop
     * @param rxPin receive pin (RX), alternative function (see data sheet) and configuration (e.g. PULL_UP, INVERT, can be NONE)
     * @param txPin transmit pin (TX), alternative function (see data sheet) and configuration (e.g. INVERT, can be NONE)
     * @param dePin driver enable pin (DE), and alternative function (see data sheet) and configuration (e.g. INVERT)
     * @param clock peripheral clock frequency (e.g. APB1_CLOCK)
     * @param uartInfo info of USART/UART instance to use
     * @param dmaInfo info of DMA channels to use
     * @param config configuration, see usart::Config. Don't forget to set ENABLE_RTS or ENABLE_DE
     * @param baudRate baud rate (e.g. 38400)
     * @param rxTimeout receiver timeout in bit times
     */
    Uart_UART_DMA(Loop_Queue &loop, gpio::Config rxPin, gpio::Config txPin, gpio::Config dePin,
        Hertz<> clock, const usart::Info &uartInfo, const dma::Info2 &dmaInfo,
        usart::Config config, int baudRate, int rxTimeout)
        : Uart_UART_DMA(loop, rxPin, txPin, clock, uartInfo, dmaInfo, config, baudRate, rxTimeout, configDe(dePin))
    {}

    /**
     * Constructor
     * @param loop event loop
     * @param rxPin receive pin (RX), alternative function (see data sheet) and configuration (e.g. PULL_UP, INVERT, can be NONE)
     * @param txPin transmit pin (TX), alternative function (see data sheet) and configuration (e.g. INVERT, can be NONE)
     * @param rtsPin ready to send pin (RTS), alternative function (see data sheet) and configuration (can be NONE)
     * @param ctsPin clear to send pin (CTS), alternative function (see data sheet) and configuration (e.g. PULL_UP, can be NONE)
     * @param clock peripheral clock frequency (e.g. APB1_CLOCK)
     * @param uartInfo info of USART/UART instance to use
     * @param dmaInfo info of DMA channels to use
     * @param config configuration, see usart::Config. Don't forget to set ENABLE_RTS and ENABLE_CTS
     * @param baudRate baud rate (e.g. 38400)
     * @param rxTimeout receiver timeout in bit times
     */
    Uart_UART_DMA(Loop_Queue &loop, gpio::Config rxPin, gpio::Config txPin,
        gpio::Config rtsPin, gpio::Config ctsPin,
        Hertz<> clock, const usart::Info &uartInfo, const dma::Info2 &dmaInfo, usart::Config config,
        int baudRate, int rxTimeout)
        : Uart_UART_DMA(loop, rxPin, txPin, clock, uartInfo, dmaInfo, config, baudRate, rxTimeout,
        configRtsCts(rtsPin, ctsPin))
    {}

    ~Uart_UART_DMA() override;

    class BufferBase;

    // Device methods
    //StateTasks<const State, Events> &getStateTasks() override;

    // BufferDevice methods
    int getBufferCount() override;
    BufferBase &getBuffer(int index) override;

    // Uart methods
    // Note: Setting baud rate and format only works when no read/write operations are in progress
    void setValue(int id, int value) override;
    int getValue(int id) override;


    // internal buffer base class, derives from IntrusiveListNode for the list of buffers and Loop_Queue::Handler2 to be notified from the event loop
    class BufferBase : public coco::Buffer, public IntrusiveListNode, public Loop_Queue::Handler {
        friend class Uart_UART_DMA;
    public:
        /**
         * Constructor
         * @param data data of the buffer
         * @param capacity capacity of the buffer
         * @param channel channel to attach to
         */
        BufferBase(uint8_t *data, int size, Uart_UART_DMA &device);
        ~BufferBase() override;

        // Buffer methods
        bool start(Op op) override;
        bool cancel() override;

    protected:
        void handle() override;

        Uart_UART_DMA &device;
        Op op;
    };

    /**
        Buffer for transferring data over UART.
        @tparam C capacity of buffer
    */
    template <int C>
    class Buffer : public BufferBase {
    public:
        Buffer(Uart_UART_DMA &device) : BufferBase(data, C, device) {}

    protected:
        alignas(4) uint8_t data[C];
    };

    /**
     * UART interrupt handler, needs to be called from USART/UART interrupt handler (e.g. USART1_IRQHandler() for usart::USART1_INFO on STM32G4)
     */
    void UART_IRQHandler() {
        uint32_t isr = this->uart->ISR;

        // check if receive timed out
        if ((isr & USART_ISR_RTOF) != 0)
            handleRx();

        // check if transmission has completed
        if ((isr & USART_ISR_TC) != 0)
            handleTx();
    }

    /**
     * DMA interrupt handler, needs to be called from Rx DMA channel interrupt handler (e.g. DMA1_Channel1_IRQHandler() for dma::DMA1_CH1_CH2_INFO on STM32G4)
     */
    void DMA_Rx_IRQHandler() {
        // check if receive has completed
        if ((this->rxStatus.get() & dma::Status::Flags::TRANSFER_COMPLETE) != 0)
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

    Loop_Queue &loop;

    // peripheral clock
    Hertz<> clock;

    // uart
    USART_TypeDef *uart;
    int uartIrq;

    // dma
    dma::Status rxStatus;
    dma::Channel rxChannel;
    int rxDmaIrq;
    dma::Channel txChannel;

    // device state
    //StateTasks<State, Events> st = State::READY;

    // list of buffers
    IntrusiveList<BufferBase> buffers;

    // list of active transfers
    InterruptQueue<BufferBase> receiveTransfers;
    InterruptQueue<BufferBase> sendTransfers;
};

} // namespace coco
