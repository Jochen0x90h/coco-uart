#include "Uart_io_uring.hpp"
#include <coco/bits.hpp>
#include <iostream>
#include <filesystem>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <asm/termbits.h> // termios2
#include <unistd.h>


namespace coco {

Uart_io_uring::~Uart_io_uring() {
    ::close(com_);
}

bool Uart_io_uring::open(String name, Format format, int baudRate, Milliseconds<> rxTimeout) {
    if (com_ != INVALID_HANDLE_VALUE)
        return false;

    // open file
    std::string n(name);
    int com = ::open(n.c_str(), O_RDWR | O_NOCTTY);
    if (com == INVALID_HANDLE_VALUE) {
        int error = errno;
        setSystemError(error);
        return false;
    }
    com_ = com;
    setSuccess();

    // configure
    setFormat(format); // also sets timeouts
    setBaudRate(baudRate);
    rxTimeout_ = max(rxTimeout, 20ms);

    // set state
    state_ = State::READY;

    // enable buffers
    for (auto &buffer : buffers_) {
        buffer.setSuccess(0);
        buffer.setReady();
    }

    // resume all coroutines waiting for state change
    notify(Events::ENTER_OPENING | Events::ENTER_READY);

    return true;
}

void Uart_io_uring::setValue(int id, int value) {
    switch (id) {
    case Value::FORMAT:
        {
            int dataBits = extract(value, int(Format::DATA_MASK));
            auto parity = Format(value) & Format::PARITY_MASK;
            int stopBits = extract(value, int(Format::STOP_MASK));

            int cflag = CREAD | CLOCAL; // enable reading, ignore modem ctrl
            switch (dataBits) {
            case 5:
                cflag |= CS5;
                break;
            case 6:
                cflag |= CS6;
                break;
            case 7:
                cflag |= CS7;
                break;
            default:
                cflag |= CS8;
            }
            switch (parity) {
            case Format::PARITY_ODD:
                cflag |= PARENB | PARODD;
            case Format::PARITY_EVEN:
                cflag |= PARENB;
            case Format::PARITY_MARK:
                cflag |= PARENB | CMSPAR | PARODD;
            case Format::PARITY_SPACE:
                cflag |= PARENB | CMSPAR;
            default:
                ;
            }
            if (stopBits != 0) {
                cflag |= CSTOPB;
            }

            // set flags
            termios2 tio;
            ioctl(com_, TCGETS2, &tio);
            tio.c_cflag = (tio.c_cflag & CBAUD) | cflag;
            tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // raw input
            tio.c_oflag &= ~OPOST;                           // raw output
            tio.c_iflag &= ~(IXON | IXOFF | IXANY);          // no software flow control
            
            // also set timeouts
            tio.c_cc[VMIN]  = 0; // wait for first character
            tio.c_cc[VTIME] = 0; // timeout
            ioctl(com_, TCSETS2, &tio);
        }
        break;
    case Value::BAUD:
        {
            baudRate_ = value;
            
            termios2 tio;
            ioctl(com_, TCGETS2, &tio);
            tio.c_cflag = (tio.c_cflag & ~CBAUD) | BOTHER; // set other baud rate
            tio.c_ispeed = value;
            tio.c_ospeed = value;
            ioctl(com_, TCSETS2, &tio);
        }
        break;
    case Value::RX_TIMEOUT:
        {
            // calc timeout in milliseconds
            rxTimeout_ = std::max(value * 1000 / baudRate_ + 1, 20) * 1ms;
        }
        break;
    case Value::OUTPUT_SIGNALS:
        {
            bool dtr = (OutputSignals(value) & OutputSignals::DTR) != 0;
            bool rts = (OutputSignals(value) & OutputSignals::RTS) != 0;
        }
        break;
    }
}

int Uart_io_uring::getValue(int id) {
    switch (id) {
    case Value::INPUT_SIGNALS:
        {

            auto result = InputSignals::NONE;

            return int(result);
        }
    }
    return 0;
}

int Uart_io_uring::getBufferCount() {
    return buffers_.count();
}

Uart_io_uring::Buffer &Uart_io_uring::getBuffer(int index) {
    return buffers_.get(index);
}

// todo: test what happens when buffers are busy when we call close()
void Uart_io_uring::close() {
    if (com_ == INVALID_HANDLE_VALUE)
        return;

    // close file
    ::close(com_);
    com_ = INVALID_HANDLE_VALUE;
    setSuccess();

    // set state
    state_ = State::DISABLED;

    // disable buffers
    for (auto &buffer : buffers_) {
        buffer.setDisabled();
    }

    // resume all coroutines waiting for state change
    notify(Events::ENTER_CLOSING | Events::ENTER_DISABLED);
}

void Uart_io_uring::onCompletion(io_uring_cqe &cqe, int id) {
    if (cqe.res & POLLIN) {
        auto buffer = receiveTransfers_.popIf(
            [this](auto &buffer) {
                int count = read(com_, buffer.data_ + receivedSize_, buffer.size_ - receivedSize_);
                receivedSize_ += count;
                return receivedSize_ >= buffer.size_;
            });
        if (!receiveTransfers_.empty()) {
            // poll again if there are more receive transfers waiting
            loop_.poll(*this, com_, POLLIN);
            loop_.invoke(*this, rxTimeout_);
        }
        if (buffer != nullptr) {
            buffer->setSuccess();
            receivedSize_ = 0;
            buffer->setReady();
        }
    }
}

void Uart_io_uring::onTimeout() {
    auto buffer = receiveTransfers_.pop();
    if (buffer != nullptr) {
        buffer->setSuccess(receivedSize_);
        receivedSize_ = 0;
        buffer->setReady();
    }
}

// Uart_io_uring::Buffer

Uart_io_uring::Buffer::Buffer(Uart_io_uring &device, int size)
    : coco::Buffer(new uint8_t[size], size, device.state_)
    , device_(device)
{
    device.buffers_.add(*this);
}

Uart_io_uring::Buffer::~Buffer() {
    delete [] data_;
}

bool Uart_io_uring::Buffer::start() {
    if (state_ != State::READY || (op_ & Op::READ_WRITE) == 0 || size_ == 0) {
        assert(state_ != State::BUSY);
        setSuccess(0);
        return false;
    }
    auto &device = device_;

    steps_ = int(op_ & Op::READ_WRITE);
    
    if ((op_ & Op::WRITE) == 0) {
        // read
        if (device.receiveTransfers_.push(*this))
            device.loop_.poll(device, device.com_, POLLIN);
    } else {
        // write
        if (!device.loop_.transfer(*this, IORING_OP_WRITE, device.com_, 0, data_, size_)) {
            // error: submit buffer full
            setError(std::errc::resource_unavailable_try_again);
            return false;
        }
    }

    // set state
    setBusy();

    return true;
}

bool Uart_io_uring::Buffer::cancel() {
    if (state_ != State::BUSY)
        return false;

    if (steps_ != 0) {
        if (!device_.loop_.cancel(*this)) {
            // error: submit buffer full
            setError(std::errc::resource_unavailable_try_again);
            return false;
        }
        steps_ = 0;
    }
    return true;
}

void Uart_io_uring::Buffer::onCompletion(io_uring_cqe &cqe, int id) {
    auto result = cqe.res;
    if (result >= 0) {
        // success
        if (steps_ == int(Op::READ_WRITE)) {
            // read after write
            steps_ = int(Op::READ);

            auto &device = device_;
            if (device.receiveTransfers_.push(*this))
                device.loop_.poll(device, device.com_, POLLIN);
        } else {
            // set success with transferred size
            setSuccess(result);
        }
    } else {
        // error
        // ERROR_OPERATION_ABORTED: cancelled
        auto error = -result;
        setSystemError(error);
    }

    // transfer finished
    setReady();
}

} // namespace coco
