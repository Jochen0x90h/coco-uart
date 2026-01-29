#include "Uart_UARTE_TIMER.hpp"
#include <coco/bits.hpp>
#include <coco/debug.hpp>
#include <coco/platform/platform.hpp>
#include <coco/platform/nvic.hpp>


namespace coco {

Uart_UARTE_TIMER::Uart_UARTE_TIMER(Loop_Queue &loop, gpio::Config rxPin, gpio::Config txPin,
    const UartInfo &uartInfo, const timer::Info &timerInfo, ppi::DualChannel ppiChannels,
    uart::Config config, uart::Format format, int baudRate, int rxTimeout)
    : Uart(State::READY)
    , loop_(loop), baudRate_(baudRate)
{
    // configure UART
    auto uart = uart_ = uartInfo.instance()
        .enable(uartInfo.enableRxTxPins(rxPin, txPin, config),
            format,
            baudRate * 1Hz,
            uart::Interrupt::ENDRX | uart::Interrupt::ENDTX);
    uartIrq_ = uartInfo.irq;
    nvic::setPriority(uartIrq_, nvic::Priority::MEDIUM); // interrupt gets enabled in first call to start()

    // configure TIMER
    auto timer = timer_ = timerInfo.timer;
    timer->BITMODE = N(TIMER_BITMODE_BITMODE, 32Bit);
    timer->PRESCALER = V(TIMER_PRESCALER_PRESCALER, 4); // 1MHz
    timer->CC[0] = (int64_t(1000000) * rxTimeout) / baudRate; // convert bit times to us
    timer->SHORTS = N(TIMER_SHORTS_COMPARE0_STOP, Enabled) | N(TIMER_SHORTS_COMPARE0_CLEAR, Enabled);

    // configure PPI
    int resetIndex = int(ppiChannels) & 0xf;
    int timeoutIndex = (int(ppiChannels) >> 8) & 0xf;

    // reset timer when a character was received
    NRF_PPI->CH[resetIndex].EEP = uintptr_t(&uart->EVENTS_RXDRDY);
    NRF_PPI->CH[resetIndex].TEP = uintptr_t(&timer->TASKS_CLEAR);
    NRF_PPI->FORK[resetIndex].TEP = uintptr_t(&timer->TASKS_START);

    // stop receiving on timeout
    NRF_PPI->CH[timeoutIndex].EEP = uintptr_t(&timer->EVENTS_COMPARE[0]);
    NRF_PPI->CH[timeoutIndex].TEP = uintptr_t(&uart->TASKS_STOPRX);

    // set enable flags
    if (rxTimeout > 0)
        NRF_PPI->CHENSET = ppiFlags_ = (1 << resetIndex) | (1 << timeoutIndex);

    // clear interrupt flags
    uart->EVENTS_ENDRX = 0;
    uart->EVENTS_ENDTX = 0;
    nvic::clear(uartIrq_);
}

Uart_UARTE_TIMER::~Uart_UARTE_TIMER() {
}

int Uart_UARTE_TIMER::getBufferCount() {
    return buffers_.count();
}

Uart_UARTE_TIMER::BufferBase &Uart_UARTE_TIMER::getBuffer(int index) {
    return buffers_.get(index);
}

void Uart_UARTE_TIMER::setValue(int id, int value) {
    switch (id) {
    case Value::FORMAT:
        // set format
        {
            //auto dataBits = Format(value) & Format::DATA_MASK;
            auto parity = Format(value) & Format::PARITY_MASK;
            auto stopBits = Format(value) & Format::STOP_MASK;

            auto format = uart::Format::NONE;

            switch (parity) {
            case Format::PARITY_EVEN:
                format |= uart::Format::PARITY_EVEN;
                break;
            default:
                format |= uart::Format::PARITY_NONE;
            }

            switch (stopBits) {
            case Format::STOP_2:
                format |= uart::Format::STOP_2;
                break;
            default:
                format |= uart::Format::STOP_1;
            }

            uart_.setFormat(format);

/*
            auto format = Format(value);
            auto parity = format & Format::PARITY_MASK;
            auto stopBits = format & Format::STOP_MASK;
            uint32_t mask = N(UARTE_CONFIG_PARITY, Msk) | N(UARTE_CONFIG_STOP, Msk);
            uart_->CONFIG = (uart_->CONFIG & ~mask)
                | (parity == Format::PARITY_EVEN ? N(UARTE_CONFIG_PARITY, Included) : N(UARTE_CONFIG_PARITY, Excluded))
                | (stopBits == Format::STOP_2 ? N(UARTE_CONFIG_STOP, Two) : N(UARTE_CONFIG_STOP, One));
*/
        }
        break;
    case Value::BAUD:
        // set baud rate
        {
            nvic::Guard gurad(uartIrq_);
            if (sendTransfers_.empty()) {
                // no send transfer in progress: apply immediately
                uart_.setBaudRate(value * 1Hz);
            } else {
                // store new baud rate to apply it after send transfers
                newBaudRate_ = value;
            }
        }
        /*{
            baudRate_ = value;
            int br = (int64_t(value) << 32) / 16000000;
            uart_->BAUDRATE = (br + 0x800) & 0xFFFFF000;
        }*/
        break;
    case Value::RX_TIMEOUT:
        timer_->CC[0] = (int64_t(1000000) * value) / baudRate_; // convert bit times to us
        if (value > 0)
            NRF_PPI->CHENSET = ppiFlags_;
        else
            NRF_PPI->CHENCLR = ppiFlags_;
        break;
    }
}

int Uart_UARTE_TIMER::getValue(int id) {
    return 0;
}

// called from UART interrupt
void Uart_UARTE_TIMER::UARTE_IRQHandler() {
    auto uart = uart_;

    // check if receive has completed or timed out
    if (uart->EVENTS_ENDRX) {
        // clear interrupt flag
        uart->EVENTS_ENDRX = 0;

        receiveTransfers_.pop(
            [this, uart](BufferBase &buffer) {
                buffer.size_ = uart->RXD.AMOUNT;
                loop_.push(buffer);
                return true;
            },
            [](BufferBase &next) {
                // start next buffer
                next.startRx();
            }
        );
    }

    // check if transmission has completed
    if (uart->EVENTS_ENDTX) {
        // clear interrupt flag
        uart->EVENTS_ENDTX = 0;

        int result = sendTransfers_.pop(
            [this](BufferBase &buffer) {
                if ((buffer.op_ & BufferBase::Op::READ) != 0) {
                    // read after write
                    buffer.op_ &= ~BufferBase::Op::WRITE;

                    // add to list of pending receive transfers and start immediately if list was empty
                    if (receiveTransfers_.push(buffer))
                        buffer.startRx();
                } else {
                    // pass buffer to event loop so that application gets notified
                    loop_.push(buffer);
                }
                return true;
            },
            [](BufferBase &next) {
                // start next buffer
                next.startTx();
            }
        );
        if (result != 2 && newBaudRate_ > 0) {
            uart.setBaudRate(newBaudRate_ * 1Hz);
            newBaudRate_ = 0;
        }
    }
}


// Uart_UARTE_TIMER::BufferBase

Uart_UARTE_TIMER::BufferBase::BufferBase(uint8_t *data, int capacity, Uart_UARTE_TIMER &device)
    : coco::Buffer(data, capacity, BufferBase::State::READY), device_(device)
{
    device.buffers_.add(*this);
}

Uart_UARTE_TIMER::BufferBase::~BufferBase() {
}

bool Uart_UARTE_TIMER::BufferBase::start(Op op) {
    if (st.state != State::READY) {
        assert(st.state != State::BUSY);
        return false;
    }

    // check if READ or WRITE flag is set
    assert((op & Op::READ_WRITE) != 0);

    op_ = op;
    auto &device = device_;
    if ((op & Op::WRITE) == 0) {
        // read

        // add to list of pending transfers and start immediately if list was empty
        if (device.receiveTransfers_.push(nvic::Guard(device.uartIrq_), *this))
            startRx();
    } else {
        // write

        // add to list of pending transfers and start immediately if list was empty
        if (device.sendTransfers_.push(nvic::Guard(device.uartIrq_), *this))
            startTx();
    }

    // set state
    setBusy();

    return true;
}

bool Uart_UARTE_TIMER::BufferBase::cancel() {
    if (st.state != State::BUSY)
        return false;
    auto &device = device_;

    if ((op_ & Op::WRITE) != 0) {
        // write

        // remove read flag in case there is a read after write
        op_ &= ~Op::READ;

        // remove from pending transfers if not yet started, otherwise complete normally
        if (device.sendTransfers_.remove(nvic::Guard(device.uartIrq_), *this, false)) {
            // cancel succeeded: set buffer ready again
            // resume application code, therefore interrupt should be enabled at this point
            setReady(0);
        }
    } else {
        // read

        // remove from pending transfers if not yet started or nothing received yet, otherwise complete normally
        if (device.receiveTransfers_.remove(nvic::Guard(device.uartIrq_), *this, false)) {
            // cancel succeeded: set buffer ready again
            // resume application code, therefore interrupt should be enabled at this point
            setReady(0);
        } else if (!device.uart_->EVENTS_RXDRDY) {
            // haven't received anything yet: stop
            device.uart_.stopRx();//->TASKS_STOPRX = TRIGGER;
        }
    }
    return true;
}

void Uart_UARTE_TIMER::BufferBase::startRx() {
    auto uart = device_.uart_;

    // set data
    volatile uint8_t *data = data_;
    uart.setRxData(data, size_);

    // clear RXDRDY
    uart->EVENTS_RXDRDY = 0;
    //uart.clearRxDataReady();

    // start UART
    uart.startRx();

    // -> UARTE_IRQHandler
}

void Uart_UARTE_TIMER::BufferBase::startTx() {
    auto uart = device_.uart_;

    // set data
    volatile uint8_t *data = data_;
    uart.setTxData(data, size_);

    // start
    uart.startTx();

    // -> UARTE_IRQHandler
}

void Uart_UARTE_TIMER::BufferBase::handle() {
    setReady();
}

} // namespace coco
