#include "Uart_UARTE_TIMER.hpp"
#include <coco/bits.hpp>
#include <coco/debug.hpp>
#include <coco/platform/platform.hpp>
#include <coco/platform/nvic.hpp>


namespace coco {

Uart_UARTE_TIMER::Uart_UARTE_TIMER(Loop_Queue &loop, gpio::Config rxPin, gpio::Config txPin,
    const uart::InfoE &uartInfo, const timer::Info &timerInfo, ppi::DualChannel ppiChannels, uart::Config config,
    int baudRate, int rxTimeout)
    : Uart(State::READY)
    , loop(loop), baudRate(baudRate)
{
    // configure UART pins
    if (rxPin != gpio::Config::NONE) {
        gpio::configureAlternate(rxPin);
        uart->PSEL.RXD = gpio::getPinIndex(rxPin);
    }
    if (txPin != gpio::Config::NONE) {
        gpio::configureAlternate(txPin);
        uart->PSEL.TXD = gpio::getPinIndex(txPin);
    }

    // configure UART
    auto uart = this->uart = uartInfo.uart;
    uart->INTENSET = N(UARTE_INTENSET_ENDRX, Set) | N(UARTE_INTENSET_ENDTX, Set);
    this->uartIrq = uartInfo.irq;
    nvic::setPriority(this->uartIrq, nvic::Priority::MEDIUM); // interrupt gets enabled in first call to start()

    // set baud rate
    // https://devzone.nordicsemi.com/f/nordic-q-a/391/uart-baudrate-register-values
    int br = (int64_t(baudRate) << 32) / 16000000;
    uart->BAUDRATE = (br + 0x800) & 0xFFFFF000;

    // set config (parity, stop bits, flow control)
    uart->CONFIG = int(config);

    // configure TIMER
    auto timer = this->timer = timerInfo.timer;
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

    // set enable fags
    if (rxTimeout > 0)
        NRF_PPI->CHENSET = this->ppiFlags = (1 << resetIndex) | (1 << timeoutIndex);

    // enable UART
    uart->ENABLE = N(UARTE_ENABLE_ENABLE, Enabled);
}

Uart_UARTE_TIMER::~Uart_UARTE_TIMER() {
}

//StateTasks<const Device::State, Device::Events> &Uart_UARTE_TIMER::getStateTasks() {
//	return makeConst(this->st);
//}

int Uart_UARTE_TIMER::getBufferCount() {
    return this->buffers.count();
}

Uart_UARTE_TIMER::BufferBase &Uart_UARTE_TIMER::getBuffer(int index) {
    return this->buffers.get(index);
}

void Uart_UARTE_TIMER::setValue(int id, int value) {
    switch (id) {
    case Value::FORMAT:
        {
            auto format = Format(value);
            auto parity = format & Format::PARITY_MASK;
            auto stopBits = format & Format::STOP_MASK;
            uint32_t mask = N(UARTE_CONFIG_PARITY, Msk) | N(UARTE_CONFIG_STOP, Msk);
            this->uart->CONFIG = (this->uart->CONFIG & ~mask)
                | (parity == Format::PARITY_EVEN ? N(UARTE_CONFIG_PARITY, Included) : N(UARTE_CONFIG_PARITY, Excluded))
                | (stopBits == Format::STOP_2 ? N(UARTE_CONFIG_STOP, Two) : N(UARTE_CONFIG_STOP, One));
        }
        break;
    case Value::BAUD:
        {
            this->baudRate = value;
            int br = (int64_t(value) << 32) / 16000000;
            this->uart->BAUDRATE = (br + 0x800) & 0xFFFFF000;
        }
        break;
    case Value::RX_TIMEOUT:
        this->timer->CC[0] = (int64_t(1000000) * value) / this->baudRate; // convert bit times to us
        if (value > 0)
            NRF_PPI->CHENSET = this->ppiFlags;
        else
            NRF_PPI->CHENCLR = this->ppiFlags;
        break;
    }
}

int Uart_UARTE_TIMER::getValue(int id) {
    return 0;
}

// called from UART interrupt
void Uart_UARTE_TIMER::UARTE_IRQHandler() {
    auto uart = this->uart;

    // check if receive has completed or timed out
    if (uart->EVENTS_ENDRX) {
        // clear interrupt flag
        uart->EVENTS_ENDRX = 0;

        this->receiveTransfers.pop(
            [this, uart](BufferBase &buffer) {
                buffer.p.size = uart->RXD.AMOUNT;
                this->loop.push(buffer);
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

        this->sendTransfers.pop(
            [this](BufferBase &buffer) {
                if ((buffer.op & BufferBase::Op::READ) != 0) {
                    // read after write
                    buffer.op &= ~BufferBase::Op::WRITE;

                    // add to list of pending receive transfers and start immediately if list was empty
                    if (this->receiveTransfers.push(buffer))
                        buffer.startRx();
                } else {
                    // pass buffer to event loop so that application gets notified
                    this->loop.push(buffer);
                }
                return true;
            },
            [](BufferBase &next) {
                // start next buffer
                next.startTx();
            }
        );
    }
}


// BufferBase

Uart_UARTE_TIMER::BufferBase::BufferBase(uint8_t *data, int size, Uart_UARTE_TIMER &device)
    : coco::Buffer(data, size, BufferBase::State::READY), device(device)
{
    device.buffers.add(*this);
}

Uart_UARTE_TIMER::BufferBase::~BufferBase() {
}

bool Uart_UARTE_TIMER::BufferBase::start(Op op) {
    if (this->st.state != State::READY) {
        assert(this->st.state != State::BUSY);
        return false;
    }

    // check if READ or WRITE flag is set
    assert((op & Op::READ_WRITE) != 0);

    this->op = op;
    auto &device = this->device;
    if ((op & Op::WRITE) == 0) {
        // read

        // add to list of pending transfers and start immediately if list was empty
        if (device.receiveTransfers.push(nvic::Guard(device.uartIrq), *this))
            startRx();
    } else {
        // write

        // add to list of pending transfers and start immediately if list was empty
        if (device.sendTransfers.push(nvic::Guard(device.uartIrq), *this))
            startTx();
    }

    // set state
    setBusy();

    return true;
}

bool Uart_UARTE_TIMER::BufferBase::cancel() {
    if (this->st.state != State::BUSY)
        return false;
    auto &device = this->device;

    if ((this->op & Op::WRITE) != 0) {
        // write

        // remove read flag in case there is a read after write
        this->op &= ~Op::READ;

        // remove from pending transfers if not yet started, otherwise complete normally
        if (device.sendTransfers.remove(nvic::Guard(device.uartIrq), *this, false)) {
            // cancel succeeded: set buffer ready again
            // resume application code, therefore interrupt should be enabled at this point
            setReady(0);
        }
    } else {
        // read

        // remove from pending transfers if not yet started or nothing received yet, otherwise complete normally
        if (device.receiveTransfers.remove(nvic::Guard(device.uartIrq), *this, false)) {
            // cancel succeeded: set buffer ready again
            // resume application code, therefore interrupt should be enabled at this point
            setReady(0);
        } else if (!device.uart->EVENTS_RXDRDY) {
            // haven't received anything yet: stop
            device.uart->TASKS_STOPRX = TRIGGER;
        }
    }
    return true;
}

void Uart_UARTE_TIMER::BufferBase::startRx() {
    auto &device = this->device;
    auto uart = device.uart;

    // set data
    uart->RXD.PTR = uintptr_t(this->p.data);
    uart->RXD.MAXCNT = this->p.capacity;

    // clear RXDRDY
    uart->EVENTS_RXDRDY = 0;

    // start UART
    uart->TASKS_STARTRX = TRIGGER;
}

void Uart_UARTE_TIMER::BufferBase::startTx() {
    auto &device = this->device;
    auto uart = device.uart;

    // set data
    uart->TXD.PTR = uintptr_t(this->p.data);
    uart->TXD.MAXCNT = this->p.size;

    // start
    uart->TASKS_STARTTX = TRIGGER;
}

void Uart_UARTE_TIMER::BufferBase::handle() {
    setReady();
}

} // namespace coco
