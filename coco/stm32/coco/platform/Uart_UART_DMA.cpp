#include "Uart_UART_DMA.hpp"
//#include <coco/debug.hpp>


namespace coco {

// Uart_UART_DMA

Uart_UART_DMA::Uart_UART_DMA(Loop_Queue &loop, gpio::Config rxPin, gpio::Config txPin,
    Hertz<> clock, const usart::Info &uartInfo, const dma::Info2 &dmaInfo,
    usart::Config config, int baudRate, int rxTimeout, uint32_t cr3)
    : Uart(State::READY)
    , loop(loop)
    , clock(clock)
{
    // enable clocks (note two cycles wait time until peripherals can be accessed, see STM32G4 reference manual section 7.2.17)
    uartInfo.rcc.enableClock();
    dmaInfo.rcc.enableClock();

    // configure UART pins
    if (rxPin != gpio::Config::NONE)
        gpio::configureAlternateInput(rxPin);
    if (txPin != gpio::Config::NONE)
        gpio::configureAlternate(txPin);

    // configure UART
    auto uart = this->uart = uartInfo.usart;
    uart->CR2 = usart::CR2(config)
        | ((rxPin & gpio::Config::INVERT) != 0 ? USART_CR2_RXINV : 0) // invert RX
        | ((txPin & gpio::Config::INVERT) != 0 ? USART_CR2_TXINV : 0) // invert TX
        | (rxTimeout > 0 ? USART_CR2_RTOEN : 0); // enable receiver timeout
    uart->CR3 = cr3
        | USART_CR3_DMAR | USART_CR3_DMAT; // DMA mode
    this->uartIrq = uartInfo.irq;
    nvic::setPriority(this->uartIrq, nvic::Priority::MEDIUM); // interrupt gets enabled in first call to start()

    // set baud rate
    uart->BRR = (int(this->clock) + (baudRate >> 1)) / baudRate;

    // set receiver timeout
    uart->RTOR = rxTimeout;

    // configure RX DMA channel
    this->rxStatus = dmaInfo.status1();
    this->rxChannel = dmaInfo.channel1();
    this->rxChannel.setPeripheralAddress(&uart->RDR);
    this->rxDmaIrq = dmaInfo.irq1;
    nvic::setPriority(this->rxDmaIrq, nvic::Priority::MEDIUM);

    // configure TX DMA channel
    this->txChannel = dmaInfo.channel2();
    this->txChannel.setPeripheralAddress(&uart->TDR);

    // map DMA to UART
    uartInfo.map(dmaInfo);

    // enable UART and transmitter
    uint32_t cr1 = usart::CR1(config) | USART_CR1_UE | USART_CR1_TE;
    uart->CR1 = cr1;

    // wait until transmitter enable gets acknowledged
    while ((uart->ISR & USART_ISR_TEACK) == 0);

    // clear and enable transmission complete interrupt
    uart->ICR = USART_ICR_TCCF;
    uart->CR1 = cr1 | USART_CR1_TCIE;
}

Uart_UART_DMA::~Uart_UART_DMA() {
}

//StateTasks<const Device::State, Device::Events> &Uart_UART_DMA::getStateTasks() {
//	return makeConst(this->st);
//}

int Uart_UART_DMA::getBufferCount() {
    return this->buffers.count();
}

Uart_UART_DMA::BufferBase &Uart_UART_DMA::getBuffer(int index) {
    return this->buffers.get(index);
}

void Uart_UART_DMA::setValue(int id, int value) {
    switch (id) {
    case Value::FORMAT:
    case Value::BAUD:
    case Value::RX_TIMEOUT:
        {
            auto uart = this->uart;
            uint32_t cr1 = uart->CR1;

            // disable uart
            uart->CR1 = 0;

            // configure
            if (id == Value::FORMAT) {
                // set format
                auto format = Format(value);
                auto dataBits = format & Format::DATA_MASK;
                auto parity = format & Format::PARITY_MASK;
                auto stopBits = format & Format::STOP_MASK;

                cr1 &= ~(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS);
#ifdef USART_CR1_M1
                if (dataBits == Format::DATA_7)
                    cr1 |= USART_CR1_M1;
                else if (dataBits == Format::DATA_9)
                    cr1 |= USART_CR1_M0;
#else
                if (dataBits == Format::DATA_9)
                    cr1 |= USART_CR1_M;
#endif
                if (parity != Format::PARITY_NONE) {
                    cr1 |= USART_CR1_PCE;
                    if (parity == Format::PARITY_ODD)
                        cr1 |= USART_CR1_PS;
                }
                uint32_t cr2 = uart->CR2 & ~(USART_CR2_STOP);
                if (stopBits == Format::STOP_1_5)
                    cr2 |= USART_CR2_STOP_0 | USART_CR2_STOP_1;
                else if (stopBits == Format::STOP_2)
                    cr2 |= USART_CR2_STOP_1;
                uart->CR2 = cr2;
            } else if (id == Value::BAUD) {
                // set baud rate
                uart->BRR = (int(this->clock) + (value >> 1)) / value;
            } else {
                // set receiver timeout
                uart->RTOR = value;
                uint32_t cr2 = uart->CR2 & ~(USART_CR2_RTOEN);
                if (value > 0)
                    cr2 |= USART_CR2_RTOEN;
                uart->CR2 = cr2;
            }

            // enable UART and transmitter
            uart->CR1 = (cr1 & ~USART_CR1_TCIE) | USART_CR1_UE | USART_CR1_TE;

            // wait until transmitter enable gets acknowledged
            while ((uart->ISR & USART_ISR_TEACK) == 0);

            // clear and enable transmission complete interrupt
            uart->ICR = USART_ICR_TCCF;
            uart->CR1 = cr1 | USART_CR1_UE | USART_CR1_TE | USART_CR1_TCIE;
        }
        break;
    }
}

int Uart_UART_DMA::getValue(int id) {
    return 0;
}

void Uart_UART_DMA::startRx(BufferBase &buffer) {
    auto uart = this->uart;

    // enable receiver and wait until it gets acknowledged
    auto cr1 = uart->CR1;
    uart->CR1 = cr1 | USART_CR1_RE;
    while ((uart->ISR & USART_ISR_REACK) == 0);

    // clear and enable receiver timeout interrupt
    uart->ICR = USART_ICR_RTOCF;
    uart->CR1 = cr1 | USART_CR1_RE | USART_CR1_RTOIE;

    // configure and enable DMA
    this->rxChannel.setMemoryAddress(buffer.p.data);
    this->rxChannel.setCount(buffer.p.capacity);
    this->rxChannel.enable(dma::Channel::Config::RX | dma::Channel::Config::TRANSFER_COMPLETE_INTERRUPT);
}

void Uart_UART_DMA::startTx(BufferBase &buffer) {
    // configure and enable DMA
    this->txChannel.setMemoryAddress(buffer.p.data);
    this->txChannel.setCount(buffer.p.size);
    this->txChannel.enable(dma::Channel::Config::TX);
}

void Uart_UART_DMA::endTx() {
}

void Uart_UART_DMA::disableRx() {
    nvic::Guard2 guard(this->uartIrq, this->rxDmaIrq);

    // check if a receive transfer is in progress
    this->receiveTransfers.pop(
        [this](Uart_UART_DMA::BufferBase &buffer) {

            // check if we are still waiting for the first character
            if (this->rxChannel.count() == int(buffer.p.capacity)) {
                // keep receive buffer
                return false;
            }

            // abort transfer and pass buffer to the event loop so that the application gets notified
            buffer.p.size = 0;//rxBuffer.p.capacity - this->rxChannel->CNDTR;
            this->loop.push(buffer);
            return true;
        }
    );

    // disable receiver and timeout interrupt
    this->uart->CR1 = this->uart->CR1 & ~(USART_CR1_RE | USART_CR1_RTOIE);

    // disable receiver DMA
    this->rxChannel.disable();

    // clear interrupt flags at peripherals and NVIC
    this->uart->ICR = USART_ICR_RTOCF;
    this->rxStatus.clear(dma::Status::Flags::TRANSFER_COMPLETE);
    nvic::clear(this->uartIrq);
    nvic::clear(this->rxDmaIrq);
}

// called from UART or DMA interrupt
void Uart_UART_DMA::handleRx() {
    auto uart = this->uart;

    // disable receiver and timeout interrupt
    uart->CR1 = uart->CR1 & ~(USART_CR1_RE | USART_CR1_RTOIE);

    // disable DMA
    this->rxChannel.disable();

    // clear interrupt flags at UART and DMA
    uart->ICR = USART_ICR_RTOCF;
    this->rxStatus.clear(dma::Status::Flags::TRANSFER_COMPLETE);

    this->receiveTransfers.pop(
        [this](BufferBase &buffer) {
            // buffer size is number of received bytes
            buffer.p.size = buffer.p.capacity - this->rxChannel.count();

            // inform application that receive is finished
            this->loop.push(buffer);
            return true;
        },
        [this](BufferBase &next) {
            // start next buffer
            startRx(next);
        }
    );
}

// called from UART interrupt
void Uart_UART_DMA::handleTx() {
    auto uart = this->uart;

    // disable DMA
    this->txChannel.disable();

    // clear interrupt flag at UART
    uart->ICR = USART_ICR_TCCF;

    this->sendTransfers.pop(
        [this](BufferBase &buffer) {
            // notify derived class that TX has ended
            endTx();

            if ((buffer.op & BufferBase::Op::READ) != 0) {
                // read after write
                buffer.op &= ~BufferBase::Op::WRITE;

                // add to list of pending receive transfers and start immediately if list was empty
                if (this->receiveTransfers.push(buffer)) // DMA interrupt has same priority and therefore doesn't need to be disabled
                    startRx(buffer);
            } else {
                // pass buffer to event loop so that application gets notified
                this->loop.push(buffer);
            }
            return true;
        },
        [this](BufferBase &next) {
            // transmit next buffer
            startTx(next);
        }
    );
}


// BufferBase

Uart_UART_DMA::BufferBase::BufferBase(uint8_t *data, int size, Uart_UART_DMA &device)
    : coco::Buffer(data, size, BufferBase::State::READY), device(device)
{
    device.buffers.add(*this);
}

Uart_UART_DMA::BufferBase::~BufferBase() {
}

bool Uart_UART_DMA::BufferBase::start(Op op) {
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
        nvic::Guard2 gurad(device.uartIrq, device.rxDmaIrq);

        // add to list of pending transfers and start immediately if list was empty
        if (device.receiveTransfers.push(*this))
            device.startRx(*this);
    } else {
        // write

        // add to list of pending transfers and start immediately if list was empty
        // sentTransfers only gets modified from UART interrupt, therefore no need to disable dma irq
        if (device.sendTransfers.push(nvic::Guard(device.uartIrq), *this))
            device.startTx(*this);
    }

    // set state
    setBusy();

    return true;
}

bool Uart_UART_DMA::BufferBase::cancel() {
    if (this->st.state != State::BUSY)
        return false;

    //return this->device.cancel(*this);
    auto &device = this->device;

    if ((this->op & Op::WRITE) != 0) {
        // write

        // remove read flag in case there is a read after write
        this->op &= ~Op::READ;

        bool cancelled = false;
        {
            nvic::Guard2 guard(device.uartIrq, device.rxDmaIrq);

            // remove from pending transfers if not yet started, otherwise complete normally
            if (device.sendTransfers.remove(*this, false)) {
                device.endTx();
                cancelled = true;
            }
        }
        if (cancelled)
            setReady(0);
/*
        // remove from pending transfers if not yet started, otherwise complete normally
        if (device.sendTransfers.remove(nvic::Guard(device.uartIrq), *this, false)) {
            // cancel succeeded: set buffer ready again
            // resume application code, therefore interrupt should be enabled at this point
            setReady(0);
        }
*/
    } else {
        // read

        // remove from pending transfers if not yet started or nothing received yet, otherwise complete normally
        if (device.receiveTransfers.remove(nvic::Guard2(device.uartIrq, device.rxDmaIrq), *this,
            [&device](BufferBase &buffer) {
                // check if we are still waiting for the first character
                if (device.rxChannel.count() == int(buffer.p.capacity)) {
                    // disable receiver and timeout interrupt
                    device.uart->CR1 = device.uart->CR1 & ~(USART_CR1_RE | USART_CR1_RTOIE);

                    // disable DMA
                    device.rxChannel.disable();

                    // clear interrupt flags at peripherals and NVIC
                    device.uart->ICR = USART_ICR_RTOCF;
                    device.rxStatus.clear(dma::Status::Flags::TRANSFER_COMPLETE);
                    nvic::clear(device.uartIrq);
                    nvic::clear(device.rxDmaIrq);

                    return true;
                }
                return false;
            },
            [&device](BufferBase &next) {
                // start next buffer if the first buffer was removed
                device.startRx(next);
            }
            ))
        {
            // cancel succeeded: set buffer ready again
            // resume application code, therefore interrupt should be enabled at this point
            setReady(0);
        }
    }
    return true;
}

void Uart_UART_DMA::BufferBase::handle() {
    setReady();
}

} // namespace coco
