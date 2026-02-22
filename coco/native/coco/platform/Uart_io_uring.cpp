#include "Uart_io_uring.hpp"
#include <coco/bits.hpp>
#include <iostream>
#include <filesystem>
#include <fcntl.h>
#include <termios.h>
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
    int com = ::open(n.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (com == INVALID_HANDLE_VALUE) {
        int error = errno;
        setSystemError(error);
        return false;
    }
    com_ = com;
    setSuccess();

    // configure
    setFormat(format);
    setBaudRate(baudRate);

    // set timeouts
    termios tty;
    tcgetattr(com, &tty);
    cfmakeraw(&tty);
    tty.c_cc[VMIN]  = 1;
    tty.c_cc[VTIME] = 1;
    tcsetattr(com, TCSANOW, &tty);

    // set state
    state_ = State::READY;

    // enable buffers
    for (auto &buffer : buffers_) {
        buffer.setSuccess(0);
        buffer.setReady();
    }

    // resume all coroutines waiting for state change
    notify(Events::ENTER_OPENING | Events::ENTER_READY);

    //loop.poll(file, POLLIN | POLLOUT, this);

    return true;
}

void Uart_io_uring::setValue(int id, int value) {
    switch (id) {
    case Value::FORMAT:
        {
            int dataBits = extract(value, int(Format::DATA_MASK));
            int parity = extract(value, int(Format::PARITY_MASK));
            int stopBits = extract(value, int(Format::STOP_MASK));

            termios tty;
            tcgetattr(com_, &tty);
            tty.c_cflag &= ~PARENB;         // No Parity
            tty.c_cflag &= ~CSTOPB;         // 1 Stop bit
            tty.c_cflag &= ~CSIZE;
            tty.c_cflag |=  CS8;            // 8 data bits
            tty.c_cflag &= ~CRTSCTS;        // No hardware flow control
            tty.c_cflag |=  CREAD | CLOCAL; // Enable reading, ignore modem ctrl

            tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Raw input
            tty.c_oflag &= ~OPOST;                           // Raw output
            tty.c_iflag &= ~(IXON | IXOFF | IXANY);          // No SW flow control
            tcsetattr(com_, TCSANOW, &tty);
        }
        break;
    case Value::BAUD:
        {
            baudRate_ = value;

            termios tty;
            tcgetattr(com_, &tty);
            cfsetospeed(&tty, B115200);
            cfsetispeed(&tty, B115200);
            tcsetattr(com_, TCSANOW, &tty);
        }
        break;
    case Value::RX_TIMEOUT:
        {
            // calc timeout in milliseconds
            int rxTimeout = std::max(value * 1000 / baudRate_ + 1, 20);

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

void Uart_io_uring::handle(io_uring_cqe &cqe) {
    
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

    flags_ = int(op_ & Op::READ_WRITE);

    // read/write
    if (!device_.loop_.transfer((op_ & Op::WRITE) == 0 ? IORING_OP_READ : IORING_OP_WRITE,
        device_.com_, 0, data_, size_, this))
    {
        // error: submit buffer full
        setError(std::errc::resource_unavailable_try_again);
        return false;
    }

    // set state
    setBusy();

    return true;
}

bool Uart_io_uring::Buffer::cancel() {
    if (state_ != State::BUSY)
        return false;

    if (flags_ != 0) {
        if (!device_.loop_.cancel(this)) {
            // error: submit buffer full
            setError(std::errc::resource_unavailable_try_again);
            return false;
        }
        flags_ = 0;
    }
    return true;
}

void Uart_io_uring::Buffer::handle(io_uring_cqe &cqe) {
    auto result = cqe.res;
    if (result >= 0) {
        // success
        if (flags_ == int(Op::READ_WRITE)) {
            // read after write
            flags_ = int(Op::READ);

            // read
            if (!device_.loop_.transfer(IORING_OP_READ, device_.com_, 0, data_, size_, this)) {
                // error: submit buffer full
                setError(std::errc::resource_unavailable_try_again);
            } else {
                // -> handle()
                return;
            }
        } else {
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
