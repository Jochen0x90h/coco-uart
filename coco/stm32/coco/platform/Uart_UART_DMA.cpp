#include "Uart_UART_DMA.hpp"
//#include <coco/debug.hpp>
//#include <coco/StreamOperators.hpp>


namespace coco {

// Uart_UART_DMA

Uart_UART_DMA::Uart_UART_DMA(Loop_Queue &loop, gpio::Config rxPin, gpio::Config txPin,
    Hertz<> clock, const UartInfo &uartInfo, const DmaInfo &dmaInfo,
    uart::Config config, uart::Format format, int baudRate, int rxTimeout)
    : Uart(State::READY)
    , loop_(loop)
    , clock_(clock)
{
    // configure UART
    auto uart = uart_ = uartInfo.enableClock()
        .enable(uartInfo.enableRxTxPins(rxPin, txPin, config),
            format,
            clock, baudRate * 1Hz,
            uart::Interrupt::RX_TIMEOUT | uart::Interrupt::TX_COMPLETE,
            uart::DmaRequest::RX_TX)
        .setRxTimeout(rxTimeout)
        .startTx();
    uartIrq_ = uartInfo.irq;
    nvic::setPriority(uartIrq_, nvic::Priority::MEDIUM); // interrupt gets enabled in first call to start()

    // configure DMA channels
    auto [rxChannel, txChannel] = dmaInfo.enableClock<RxChannel::MODE, TxChannel::MODE>();
    rxChannel_ = rxChannel
        .configure()
        .setSourceAddress(&uart->RDR);
    rxDmaIrq_ = dmaInfo.irq1;
    nvic::setPriority(rxDmaIrq_, nvic::Priority::MEDIUM);
    txChannel_ = txChannel
        .configure()
        .setDestinationAddress(&uart->TDR);

    // map DMA to UART
    uartInfo.map(dmaInfo);

    // clear interrupt flags
    uart.clear(uart::Status::ALL);
    nvic::clear(uartIrq_);

    // uartIrq and rxDmaIrq get enabled in first call to start()
}

Uart_UART_DMA::~Uart_UART_DMA() {
}

int Uart_UART_DMA::getBufferCount() {
    return buffers_.count();
}

Uart_UART_DMA::BufferBase &Uart_UART_DMA::getBuffer(int index) {
    return buffers_.get(index);
}

void Uart_UART_DMA::setValue(int id, int value) {
    switch (id) {
    case Value::FORMAT:
        // set format
        {
            auto dataBits = Format(value) & Format::DATA_MASK;
            auto parity = Format(value) & Format::PARITY_MASK;
            auto stopBits = Format(value) & Format::STOP_MASK;

            auto format = uart::Format::NONE;

            switch (dataBits) {
#ifdef HAVE_USART_DATA_7
            case Format::DATA_7:
                format |= uart::Format::DATA_7;
                break;
#endif
            case Format::DATA_9:
                format |= uart::Format::DATA_9;
                break;
            default:
                format |= uart::Format::DATA_8;
            }

            switch (parity) {
            case Format::PARITY_EVEN:
                format |= uart::Format::PARITY_EVEN;
                break;
            case Format::PARITY_ODD:
                format |= uart::Format::PARITY_ODD;
                break;
            default:
                format |= uart::Format::PARITY_NONE;
            }

            switch (stopBits) {
            case Format::STOP_1_5:
                format |= uart::Format::STOP_1_5;
                break;
            case Format::STOP_2:
                format |= uart::Format::STOP_2;
                break;
            default:
                format |= uart::Format::STOP_1;
            }

            uart_.setFormat(format);
        }
        break;
    case Value::BAUD:
        // set baud rate
        {
            nvic::Guard2 gurad(uartIrq_, rxDmaIrq_);
            if (sendTransfers_.empty()) {
                // no send transfer in progress: apply immediately
                uart_.setBaudRate(clock_, value * 1Hz);
            } else {
                // store new baud rate to apply it after send transfers
                newBaudRate_ = value;
            }
        }
        break;
    case Value::RX_TIMEOUT:
        // set receiver timeout
        uart_.setRxTimeout(value);
        break;
    }
}

int Uart_UART_DMA::getValue(int id) {
    switch (id) {
    case Value::BAUD:
        return int(uart_.getBaudRate(clock_));
    }
    return 0;
}

void Uart_UART_DMA::startRx(BufferBase &buffer) {
    //debug::out << "startRx\n";

    // enable receiver
    uart_.startRx();

    // configure and enable DMA
    volatile void *data = buffer.data_;
    rxChannel_
        .setDestinationAddress(data)
        .setCount(buffer.capacity_)
        .enable(dma::Config::TRANSFER_COMPLETE_INTERRUPT);

    // -> handleRx
}

// called when interrupts are disalbed (via BufferBase::start() or handleTx())
void Uart_UART_DMA::startTx(BufferBase &buffer) {
    //debug::out << "startTx\n";
    // configure and enable DMA
    volatile void *data = buffer.data_;
    txChannel_
        .setSourceAddress(data)
        .setCount(buffer.size_)
        .enable();

    // -> handleTx
}

// called when interrupts are disalbed (via BufferBase::cancel() or handleTx())
void Uart_UART_DMA::endTx() {
}

// called when interrupts are disalbed (via Rs485_UART_DMA::startTx())
void Uart_UART_DMA::disableRx() {
    //debug::out << "disableRx\n";
    nvic::Guard2 guard(uartIrq_, rxDmaIrq_);

    // check if a receive transfer is in progress
    /*receiveTransfers_.pop(
        [this](Uart_UART_DMA::BufferBase &buffer) {

            // check if we are still waiting for the first character
            if (rxChannel_.count() == int(buffer.capacity_)) {
                // keep receive buffer
                return false;
            }

            // abort transfer and pass buffer to the event loop so that the application gets notified
            buffer.size_ = 0;//rxBuffer.p.capacity - rxChannel->CNDTR;
            loop_.push(buffer);
            return true;
        }
    );*/
    receiveTransfers_.popIf(
        [this](auto &buffer) {
            // keep receive buffer (reject pop) if we are still waiting for the first character
            return rxChannel_.count() != int(buffer.capacity_);
        },
        [this](auto &buffer) {
            // abort transfer and pass buffer to the event loop so that the application gets notified
            buffer.size_ = 0;//rxBuffer.p.capacity - rxChannel->CNDTR;
            loop_.push(buffer);
        }
    );


    // disable rx and DMA
    uart_
        .stopRx()
        .clear(uart::Status::RX_TIMEOUT);
    rxChannel_
        .disable()
        .clear(dma::Status::TRANSFER_COMPLETE);
    nvic::clear(uartIrq_);
    nvic::clear(rxDmaIrq_);
}

// called from UART or DMA interrupt
void Uart_UART_DMA::handleRx() {
    //debug::out << "handleRx " << dec(rxChannel.count()) << "\n";
    // disable rx and DMA
    uart_
        .stopRx()
        .clear(uart::Status::RX_TIMEOUT);
    rxChannel_
        .disable()
        .clear(dma::Status::TRANSFER_COMPLETE);

    int count = rxChannel_.count();

    /*receiveTransfers_.pop(
        [this](BufferBase &buffer) {
            // buffer size is number of received bytes
            buffer.setSuccess(buffer.capacity_ - rxChannel_.count());

            // inform application that receive is finished
            loop_.push(buffer);
            return true;
        },
        [this](BufferBase &next) {
            // start next buffer
            startRx(next);
        }
    );*/
    receiveTransfers_.pop(
        [this](auto &next) {
            // start next buffer
            startRx(next);
        },
        [this, count](auto &buffer) {
            // buffer size is number of received bytes
            buffer.setSuccess(buffer.capacity_ - count);

            // inform application that receive is finished
            loop_.push(buffer);
        });
}

// called from UART interrupt (DMA interrupt has same priority and therefore can't execute)
void Uart_UART_DMA::handleTx() {
    //debug::out << "handleTx\n";
    auto uart = uart_;

    // disable tx DMA
    txChannel_.disable();

    // clear interrupt flag at UART
    uart.clear(uart::Status::TX_COMPLETE);

    /*int result = sendTransfers_.pop(
        [this](BufferBase &buffer) {
            // notify derived class that TX has ended
            endTx();

            if ((buffer.steps_ & int(BufferBase::Op::READ)) != 0) {
                // read after write

                // update flags for cancel()
                buffer.steps_ = int(BufferBase::Op::READ);

                // add to list of pending receive transfers and start immediately if list was empty
                if (receiveTransfers_.push(buffer)) // DMA interrupt has same priority and therefore doesn't need to be disabled
                    startRx(buffer);
            } else {
                // update flags for cancel()
                buffer.steps_ = 0;

                // pass buffer to event loop so that application gets notified
                buffer.setSuccess();
                loop_.push(buffer);
            }
            return true;
        },
        [this](BufferBase &next) {
            // transmit next buffer
            startTx(next);
        }
    );
    if (result != 2 && newBaudRate_ > 0) {
        uart.setBaudRate(clock_, newBaudRate_ * 1Hz);
        newBaudRate_ = 0;
    }*/

    // notify derived class that TX has ended
    endTx();

    sendTransfers_.pop(
        [this](auto &next) {
            // transmit next buffer
            startTx(next);
        },
        [this](auto &buffer) {
            if ((buffer.steps_ & int(BufferBase::Op::READ)) != 0) {
                // read after write

                // update flags for cancel()
                buffer.steps_ = int(BufferBase::Op::READ);

                // add to list of pending receive transfers and start immediately if list was empty
                if (receiveTransfers_.push(buffer)) // DMA interrupt has same priority and therefore doesn't need to be disabled
                    startRx(buffer);
            } else {
                // update flags for cancel()
                buffer.steps_ = 0;

                // pass buffer to event loop so that application gets notified
                buffer.setSuccess();
                loop_.push(buffer);
            }
        });

    // change baud rate only if no send transfer in progress
    if (sendTransfers_.empty() && newBaudRate_ > 0) {
        uart.setBaudRate(clock_, newBaudRate_ * 1Hz);
        newBaudRate_ = 0;
    }
}


// Uart_UART_DMA::BufferBase

Uart_UART_DMA::BufferBase::BufferBase(uint8_t *data, int capacity, Uart_UART_DMA &device)
    : coco::Buffer(data, capacity, BufferBase::State::READY), device_(device)
{
    device.buffers_.add(*this);
}

Uart_UART_DMA::BufferBase::~BufferBase() {
}

bool Uart_UART_DMA::BufferBase::start() {
    if (state_ != State::READY) {
        assert(false);
        setError(std::errc::resource_unavailable_try_again);
        return false;
    }
    if ((op_ & Op::READ_WRITE) == 0 || size_ == 0) {
        setSuccess();
        return false;
    }
    auto &device = device_;

    steps_ = int(op_ & Op::READ_WRITE);

    if ((op_ & Op::WRITE) == 0) {
        // read
        nvic::Guard2 gurad(device.uartIrq_, device.rxDmaIrq_);

        // add to list of pending transfers and start immediately if list was empty
        if (device.receiveTransfers_.push(*this))
            device.startRx(*this);
    } else {
        // write

        // add to list of pending transfers and start immediately if list was empty
        if (device.sendTransfers_.push(nvic::Guard2(device.uartIrq_, device.rxDmaIrq_), *this))
            device.startTx(*this);
    }

    // set state
    setBusy();

    return true;
}

bool Uart_UART_DMA::BufferBase::cancel() {
    if (state_ != State::BUSY)
        return false;
    auto &device = device_;

    bool canceled = false;
    {
        nvic::Guard2 guard(device.uartIrq_, device.rxDmaIrq_);
        if ((steps_ & int(Op::WRITE)) != 0) {
            // write: buffer is in sendTransfers_ list

            // remove from pending transfers if not yet started, otherwise complete normally
            if (device.sendTransfers_.removeButFirst(*this)) {
                //device.endTx();
                canceled = true;
            }
        } else if ((steps_ & int(Op::READ)) != 0) {
            // read: buffer is in receiveTransfers_ list

            // remove this buffer from pending transfers if not yet started or nothing received yet, otherwise complete normally
            /*if (device.receiveTransfers_.remove(*this,
                [&device](BufferBase &buffer) {
                    // check if we are still waiting for the first character
                    if (device.rxChannel_.count() == int(buffer.capacity_)) {
                        // disable rx and DMA
                        device.uart_
                            .stopRx()
                            .clear(uart::Status::RX_TIMEOUT);
                        device.rxChannel_
                            .disable()
                            .clear(dma::Status::TRANSFER_COMPLETE);
                        nvic::clear(device.uartIrq_);
                        nvic::clear(device.rxDmaIrq_);
                        return true;
                    }
                    return false;
                },
                [&device](BufferBase &next) {
                    // start next buffer if the first buffer was removed
                    device.startRx(next);
                }
                ) == 1)
            {
                canceled = true;
            }*/

            canceled = device.receiveTransfers_.removeButFirstIf(*this,
                [&device](auto &buffer) {
                    // we can cancel if we are still waiting for the first character
                    if (device.rxChannel_.count() == int(buffer.capacity_)) {
                        // disable rx and DMA
                        device.uart_
                            .stopRx()
                            .clear(uart::Status::RX_TIMEOUT);
                        device.rxChannel_
                            .disable()
                            .clear(dma::Status::TRANSFER_COMPLETE);
                        nvic::clear(device.uartIrq_);
                        nvic::clear(device.rxDmaIrq_);
                        return true;
                    }
                    return false;
                },
                [&device](auto &next) {
                    // start next buffer if the first buffer was removed
                    device.startRx(next);
                });
        }

        // clear pending read/write operations
        steps_ = 0;
    }

    if (canceled) {
        // cancel succeeded: set buffer ready again and resume application code waiting for ready state
        setError(std::errc::operation_canceled);
        setReady();
    }

    return true;
}

void Uart_UART_DMA::BufferBase::handle() {
    setReady();
}

} // namespace coco
