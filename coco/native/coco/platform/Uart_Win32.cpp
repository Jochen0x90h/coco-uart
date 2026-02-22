#include <coco/platform/WindowsDef.hpp>
#include <Windows.h>
#include <winioctl.h>
#include <ntddser.h>
#include <coco/platform/WindowsUndef.hpp>

#include "Uart_Win32.hpp"
#include <coco/bits.hpp>
#include <iostream>
#include <filesystem>


namespace coco {

Uart_Win32::~Uart_Win32() {
    CloseHandle(file_);
}

bool Uart_Win32::open(String name, Format format, int baudRate, Milliseconds<> rxTimeout) {
    if (file_ != INVALID_HANDLE_VALUE)
        return false;

    // open file
    std::filesystem::path path(std::u8string_view(reinterpret_cast<const char8_t *>(name.data()), name.size()));
    HANDLE file = CreateFileW(path.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0, // no sharing
        nullptr, // security
        OPEN_EXISTING, // open existing port
        FILE_FLAG_OVERLAPPED,
        nullptr);
    if (file == INVALID_HANDLE_VALUE) {
        int error = GetLastError();
        setSystemError(error);
        return false;
    }

    // add file to completion port of event loop
    Loop_Win32::CompletionHandler *handler = this;
    if (CreateIoCompletionPort(
        file,
        loop_.port,
        ULONG_PTR(handler),
        0) == nullptr)
    {
        int error = GetLastError();
        setSystemError(error);
        CloseHandle(file);
        return false;
    }
    file_ = file;
    setSuccess();

    // configure
    setFormat(format);
    setBaudRate(baudRate);

    // https://learn.microsoft.com/en-us/windows/win32/api/winbase/nf-winbase-setcommtimeouts
    COMMTIMEOUTS timeouts;
    timeouts.ReadIntervalTimeout = rxTimeout.value;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 0;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 0;
    SetCommTimeouts(file_, &timeouts);


    // set state
    state_ = State::READY;

    // enable buffers
    for (auto &buffer : buffers_) {
        buffer.setSuccess(0);
        buffer.setReady();
    }

    // resume all coroutines waiting for state change
    notify(Events::ENTER_OPENING | Events::ENTER_READY);

    // set wait mask
    ULONG value = SERIAL_EV_RLSD | SERIAL_EV_DSR | SERIAL_EV_RING | SERIAL_EV_CTS;
    DWORD transferred;
    auto result = DeviceIoControl(file_,
        IOCTL_SERIAL_SET_WAIT_MASK,
        &value, 4, // input buffer
        nullptr, 0, // output buffer
        nullptr, // transfered into output buffer
        nullptr);

    // initialize overlapped
    memset(&overlapped_, 0, sizeof(OVERLAPPED));

    // wait for events
    DeviceIoControl(file_,
        IOCTL_SERIAL_WAIT_ON_MASK,
        nullptr, 0, // input buffer
        &mask_, 4, // output buffer
        nullptr, // transfered into output buffer
        &overlapped_);
    return true;
}

void Uart_Win32::setValue(int id, int value) {
    /*
        https://learn.microsoft.com/en-us/windows/win32/api/winbase/ns-winbase-dcb
        https://learn.microsoft.com/en-us/windows/win32/api/winbase/ns-winbase-commtimeouts
        http://unixwiz.net/techtips/termios-vmin-vtime.html
        https://stackoverflow.com/questions/4968529/how-can-i-set-the-baud-rate-to-307-200-on-linux
    */

    switch (id) {
    case Value::FORMAT:
        {
            int dataBits = extract(value, int(Format::DATA_MASK));
            int parity = extract(value, int(Format::PARITY_MASK));
            int stopBits = extract(value, int(Format::STOP_MASK));

            DCB dcb;
            dcb.DCBlength = sizeof(dcb);
            GetCommState(file_, &dcb);
            dcb.ByteSize = dataBits;
            dcb.fParity = parity == 0 ? 0 : 1;
            dcb.Parity = parity;
            dcb.StopBits = stopBits;
            dcb.fDtrControl = 0;
            dcb.fRtsControl = 0;
            SetCommState(file_, &dcb);
        }
        break;
    case Value::BAUD:
        {
            baudRate_ = value;

            DCB dcb;
            dcb.DCBlength = sizeof(dcb);
            GetCommState(file_, &dcb);
            dcb.BaudRate = value;
            SetCommState(file_, &dcb);
        }
        break;
    case Value::RX_TIMEOUT:
        {
            // calc timeout in milliseconds
            int rxTimeout = std::max(value * 1000 / baudRate_ + 1, 20);

            // https://learn.microsoft.com/en-us/windows/win32/api/winbase/nf-winbase-setcommtimeouts
            COMMTIMEOUTS timeouts;
            timeouts.ReadIntervalTimeout = rxTimeout;
            timeouts.ReadTotalTimeoutMultiplier = 0;
            timeouts.ReadTotalTimeoutConstant = 0;
            timeouts.WriteTotalTimeoutMultiplier = 0;
            timeouts.WriteTotalTimeoutConstant = 0;
            SetCommTimeouts(file_, &timeouts);
        }
        break;
    case Value::OUTPUT_SIGNALS:
        // http://www.ioctls.net/
        {
            bool dtr = (OutputSignals(value) & OutputSignals::DTR) != 0;
            bool rts = (OutputSignals(value) & OutputSignals::RTS) != 0;

            // RTS needs to be send first because of a bug in usbser.sys (https://answers.microsoft.com/en-us/windows/forum/all/usbsersys-does-not-handle-rts-signal-correctly/e348047e-dacd-47d5-8e74-1fd2f275bebb)
            DWORD transferred;
            DeviceIoControl(file_,
                rts ? IOCTL_SERIAL_SET_RTS : IOCTL_SERIAL_CLR_RTS,
                nullptr, 0, // input buffer
                nullptr, 0, // output buffer
                &transferred,
                nullptr);
            DeviceIoControl(file_,
                dtr ? IOCTL_SERIAL_SET_DTR : IOCTL_SERIAL_CLR_DTR,
                nullptr, 0, // input buffer
                nullptr, 0, // output buffer
                &transferred,
                nullptr);
        }
        break;
    }
}

int Uart_Win32::getValue(int id) {
    switch (id) {
    case Value::INPUT_SIGNALS:
        {
            // https://learn.microsoft.com/en-us/windows-hardware/drivers/ddi/ntddser/ni-ntddser-ioctl_serial_set_wait_mask
            DWORD status;
            DWORD transferred;
            DeviceIoControl(file_, IOCTL_SERIAL_GET_MODEMSTATUS,
                nullptr, 0, // input buffer
                &status, 4, // output buffer
                &transferred,
                nullptr);
            std::cout << "status " << status << std::endl;

            auto result = InputSignals::NONE;
            if (transferred == 4) {
                if (status & SERIAL_DCD_STATE)
                    result |= InputSignals::DCD;
                if (status & SERIAL_DSR_STATE)
                    result |= InputSignals::DSR;
                if (status & SERIAL_RI_STATE)
                    result |= InputSignals::RI;
                if (status & SERIAL_CTS_STATE)
                    result |= InputSignals::CTS;
            }
            return int(result);
        }
    }
    return 0;
}

int Uart_Win32::getBufferCount() {
    return buffers_.count();
}

Uart_Win32::Buffer &Uart_Win32::getBuffer(int index) {
    return buffers_.get(index);
}

// todo: test what happens when buffers are busy when we call close()
void Uart_Win32::close() {
    if (file_ == INVALID_HANDLE_VALUE)
        return;

    // close file
    CloseHandle(file_);
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

void Uart_Win32::handle(OVERLAPPED *overlapped) {
    for (auto &buffer : buffers_) {
        if (overlapped == &buffer.overlapped_) {
            buffer.handle(overlapped);
            return;
        }
    }
    if (overlapped = &overlapped_) {
        DWORD transferred;
        auto result = GetOverlappedResult(file_, overlapped, &transferred, false);
        if (result) {
            // success
            setSuccess();
        } else {
            // error
            auto error = GetLastError();
            setSystemError(error);
        }

        // resume all coroutines waiting for state change
        notify(Events::SIGNALS_CHANGED);

        // wait for events again
        DeviceIoControl(file_,
            IOCTL_SERIAL_WAIT_ON_MASK,
            nullptr, 0, // input buffer
            &mask_, 4, // output buffer
            nullptr, // transfered into output buffer
            &overlapped_);
    }
}


// Uart_Win32::Buffer

Uart_Win32::Buffer::Buffer(Uart_Win32 &device, int size)
    : coco::Buffer(new uint8_t[size], size, device.state_)
    , device_(device)
{
    device.buffers_.add(*this);
}

Uart_Win32::Buffer::~Buffer() {
    delete [] data_;
}

bool Uart_Win32::Buffer::start() {
    if (state_ != State::READY || (op_ & Op::READ_WRITE) == 0 || size_ == 0) {
        assert(state_ != State::BUSY);
        setSuccess(0);
        return false;
    }

    // initialize overlapped
    memset(&overlapped_, 0, sizeof(OVERLAPPED));

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

bool Uart_Win32::Buffer::cancel() {
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

void Uart_Win32::Buffer::handle(OVERLAPPED *overlapped) {
    DWORD transferred;
    auto result = GetOverlappedResult(device_.file_, overlapped, &transferred, false);
    if (result) {
        // success
        if (flags_ == int(Op::READ_WRITE)) {
            // read after write
            flags_ = int(Op::READ);

            // initialize overlapped
            memset(&overlapped_, 0, sizeof(OVERLAPPED));

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
        auto error = GetLastError();
        setSystemError(error);
    }

    // transfer finished
    setReady();
}

} // namespace coco
