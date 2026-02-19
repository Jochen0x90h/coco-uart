#include "Uart_io_uring.hpp"
#include <coco/bits.hpp>
#include <iostream>
#include <filesystem>


namespace coco {

Uart_io_uring::~Uart_io_uring() {
    CloseHandle(file_);
}

bool Uart_io_uring::open(String name, Format format, int baudRate, Milliseconds<> rxTimeout) {
    if (file_ != INVALID_HANDLE_VALUE)
        return false;

    // open file
    std::string n = name;
    int file = open(n.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (file == INVALID_HANDLE_VALUE) {
        int error = errno;
        setSystemError(error);
        return false;
    }
    file_ = file;
    setSuccess();

    // configure
    setFormat(format);
    setBaudRate(baudRate);

    // set timeouts


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
            int parity = extract(value, int(Format::PARITY_MASK));
            int stopBits = extract(value, int(Format::STOP_MASK));

            termios tty;
            tcgetattr(file_, &tty);
            tty.c_cflag &= ~PARENB;         // No Parity
            tty.c_cflag &= ~CSTOPB;         // 1 Stop bit
            tty.c_cflag &= ~CSIZE;
            tty.c_cflag |=  CS8;            // 8 data bits
            tty.c_cflag &= ~CRTSCTS;        // No hardware flow control
            tty.c_cflag |=  CREAD | CLOCAL; // Enable reading, ignore modem ctrl

            tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Raw input
            tty.c_oflag &= ~OPOST;                           // Raw output
            tty.c_iflag &= ~(IXON | IXOFF | IXANY);          // No SW flow control
            tcsetattr(file_, TCSANOW, &tty)
        }
        break;
    case Value::BAUD:
        {
            baudRate_ = value;

            termios tty;
            tcgetattr(file_, &tty);
            cfsetospeed(&tty, B115200);
            cfsetispeed(&tty, B115200);
            tcsetattr(file_, TCSANOW, &tty)
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
    if (file_ == INVALID_HANDLE_VALUE)
        return;

    // close file
    ::close(file_);
    file_ = INVALID_HANDLE_VALUE;
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


// Buffer

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

    // get data and size to read/write
    int result;
    if ((op_ & Op::WRITE) == 0) {
        // read
        result = ReadFile(device_.file_, data_, capacity_, nullptr, &overlapped_);
    } else {
        // write
        result = WriteFile(device_.file_, data_, size_, nullptr, &overlapped_);
    }

    if (!result) {
        int error = GetLastError();
        if (error != ERROR_IO_PENDING) {
            // error
            setSystemError(error);
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

    if (flags_ != 0) {
        auto result = CancelIoEx(device_.file_, &overlapped_);
        if (!result) {
            int error = GetLastError();
            setSystemError(error);
            //std::cerr << "cancel error " << e << std::endl;
            return false;
        }

        // clear pending read/write operations
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
            result = ReadFile(device_.file_, data_, capacity_, nullptr, &overlapped_);
            if (!result) {
                int error = GetLastError();
                if (error != ERROR_IO_PENDING) {
                    // error
                    setSystemError(error);
                } else {
                    // -> handle()
                    return;
                }
            } else {
                // -> handle()
                return;
            }
        } else {
            setSuccess(transferred);
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
