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

namespace {

    void setBaudRate(HANDLE com, int baudRate) {
        DCB dcb;
        dcb.DCBlength = sizeof(dcb);
        GetCommState(com, &dcb);
        dcb.BaudRate = baudRate;
        SetCommState(com, &dcb);
    }

    void setFormatAndBaudRate(HANDLE com, Uart::Format format, int baudRate) {
        int dataBits = extract(format, Uart::Format::DATA_MASK);
        int parity = extract(format, Uart::Format::PARITY_MASK);
        int stopBits = extract(format, Uart::Format::STOP_MASK);

        DCB dcb;
        dcb.DCBlength = sizeof(dcb);
        GetCommState(com, &dcb);
        dcb.ByteSize = dataBits;
        dcb.fParity = parity == 0 ? 0 : 1;
        dcb.Parity = parity;
        dcb.StopBits = stopBits;
        dcb.fDtrControl = 0;
        dcb.fRtsControl = 0;
        dcb.BaudRate = baudRate;
        SetCommState(com, &dcb);
    }

    // https://learn.microsoft.com/en-us/windows/win32/api/winbase/nf-winbase-setcommtimeouts
    void setRxTimeout(HANDLE com, int rxTimeout) {
        COMMTIMEOUTS timeouts;
        timeouts.ReadIntervalTimeout = std::max(rxTimeout, 20);
        timeouts.ReadTotalTimeoutMultiplier = 0;
        timeouts.ReadTotalTimeoutConstant = 0;
        timeouts.WriteTotalTimeoutMultiplier = 0;
        timeouts.WriteTotalTimeoutConstant = 0;
        SetCommTimeouts(com, &timeouts);
    }

} // anonymous namespace

Uart_Win32::~Uart_Win32() {
    CloseHandle(handle_);
}

bool Uart_Win32::open(const std::filesystem::path &path, Format format, int baudRate, Milliseconds<> rxTimeout) {
    if (handle_ != INVALID_HANDLE_VALUE)
        return false;

    // open com port
    HANDLE handle = CreateFileW(path.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0, // no sharing
        nullptr, // security
        OPEN_EXISTING, // open existing port
        FILE_FLAG_OVERLAPPED,
        nullptr);
    if (handle == INVALID_HANDLE_VALUE) {
        int error = GetLastError();
        setSystemError(error);
        return false;
    }

    // add file to completion port of event loop
    if (CreateIoCompletionPort(
        handle,
        loop_.port,
        ULONG_PTR(&static_cast<Loop_Win32::CompletionHandler &>(*this)),
        0) == nullptr)
    {
        int error = GetLastError();
        setSystemError(error);
        CloseHandle(handle);
        return false;
    }
    handle_ = handle;
    setSuccess();

    // configure
    setFormatAndBaudRate(handle, format, baudRate);
    coco::setRxTimeout(handle, rxTimeout.value);

    // store baud rate
    baudRate_ = baudRate;

    // store rx timeout in bit times
    rxTimeout_ = rxTimeout.value * baudRate / 1000;

    // set wait mask
    ULONG value = SERIAL_EV_RLSD | SERIAL_EV_DSR | SERIAL_EV_RING | SERIAL_EV_CTS;
    DWORD transferred;
    auto result = DeviceIoControl(handle_,
        IOCTL_SERIAL_SET_WAIT_MASK,
        &value, 4, // input buffer
        nullptr, 0, // output buffer
        nullptr, // transfered into output buffer
        nullptr);

    // initialize overlapped
    memset(&overlapped_, 0, sizeof(OVERLAPPED));

    // wait for events
    DeviceIoControl(handle_,
        IOCTL_SERIAL_WAIT_ON_MASK,
        nullptr, 0, // input buffer
        &mask_, 4, // output buffer
        nullptr, // transfered into output buffer
        &overlapped_);

    // set state
    state_ = State::READY;

    // enable buffers
    for (auto &buffer : buffers_) {
        buffer.setReady();
    }

    // resume all coroutines waiting for state change
    notify(Events::ENTER_OPENING | Events::ENTER_READY);

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
        setFormatAndBaudRate(handle_, Format(value), baudRate_);
        break;
    case Value::BAUD:
        baudRate_ = value;
        coco::setBaudRate(handle_, value);

        // set rx timeout in milliseconds
        coco::setRxTimeout(handle_, rxTimeout_ * 1000 / baudRate_ + 1);
        break;
    case Value::RX_TIMEOUT:
        // timeout in bit times
        rxTimeout_ = value;

        // set rx timeout in milliseconds
        coco::setRxTimeout(handle_, rxTimeout_ * 1000 / baudRate_ + 1);
        break;
    case Value::OUTPUT_SIGNALS:
        // http://www.ioctls.net/
        //if (file_ != INVALID_HANDLE_VALUE) {
        {
            bool dtr = (OutputSignals(value) & OutputSignals::DTR) != 0;
            bool rts = (OutputSignals(value) & OutputSignals::RTS) != 0;

            // RTS needs to be send first because of a bug in usbser.sys (https://answers.microsoft.com/en-us/windows/forum/all/usbsersys-does-not-handle-rts-signal-correctly/e348047e-dacd-47d5-8e74-1fd2f275bebb)
            DWORD transferred;
            DeviceIoControl(handle_,
                rts ? IOCTL_SERIAL_SET_RTS : IOCTL_SERIAL_CLR_RTS,
                nullptr, 0, // input buffer
                nullptr, 0, // output buffer
                &transferred,
                nullptr);
            DeviceIoControl(handle_,
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
            DeviceIoControl(handle_, IOCTL_SERIAL_GET_MODEMSTATUS,
                nullptr, 0, // input buffer
                &status, 4, // output buffer
                &transferred,
                nullptr);
            //std::cout << "status " << status << std::endl;

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
    if (handle_ == INVALID_HANDLE_VALUE)
        return;

    // close handle
    CloseHandle(handle_);
    handle_ = INVALID_HANDLE_VALUE;
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

/*
void Uart_Win32::onTimeout() {
    // open file
    //std::filesystem::path path(std::u8string_view(reinterpret_cast<const char8_t *>(name.data()), name.size()));
    HANDLE file = CreateFileW(path_.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0, // no sharing
        nullptr, // security
        OPEN_EXISTING, // open existing port
        FILE_FLAG_OVERLAPPED,
        nullptr);
    if (file == INVALID_HANDLE_VALUE) {
        int error = GetLastError();
        setSystemError(error);

        // try again
        loop_.invoke(*this, 1s);
        return;
    }

    // add file to completion port of event loop
    if (CreateIoCompletionPort(
        file,
        loop_.port,
        ULONG_PTR(&static_cast<Loop_Win32::CompletionHandler &>(*this)),
        0) == nullptr)
    {
        int error = GetLastError();
        setSystemError(error);
        CloseHandle(file);

        // try again
        loop_.invoke(*this, 1s);
        return;
    }
    file_ = file;
    setSuccess();

    // configure
    DCB dcb;
    dcb.DCBlength = sizeof(dcb);
    GetCommState(file_, &dcb);
    applyFormat(dcb, format_);
    dcb.BaudRate = baudRate_;
    SetCommState(file_, &dcb);

    // https://learn.microsoft.com/en-us/windows/win32/api/winbase/nf-winbase-setcommtimeouts
    COMMTIMEOUTS timeouts;
    timeouts.ReadIntervalTimeout = std::max(rxTimeout_.value, 20);
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 0;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 0;
    SetCommTimeouts(file_, &timeouts);

    // set wait mask
    ULONG value = SERIAL_EV_RLSD | SERIAL_EV_DSR | SERIAL_EV_RING | SERIAL_EV_CTS;
    DWORD transferred;
    auto result = DeviceIoControl(handle_,
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

    // set state
    state_ = State::READY;

    // enable buffers
    for (auto &buffer : buffers_) {
        buffer.setReady();
    }

    // resume all coroutines waiting for state change
    notify(Events::ENTER_READY);
}*/

void Uart_Win32::onCompletion(OVERLAPPED *overlapped) {
    // check for buffer transfer
    for (auto &buffer : buffers_) {
        if (overlapped == &buffer.overlapped_) {
            buffer.onCompletion(overlapped);
            return;
        }
    }

    // check for state change
    if (overlapped = &overlapped_) {
        DWORD transferred;
        auto result = GetOverlappedResult(handle_, overlapped, &transferred, false);
        if (result) {
            // success
            setSuccess();
        } else {
            // error
            // ERROR_OPERATION_ABORTED: device removed
            auto error = GetLastError();
            setSystemError(error);
            if (error == ERROR_OPERATION_ABORTED)
                close();
        }

        // resume all coroutines waiting for state change
        notify(Events::SIGNALS_CHANGED);

        // wait for events again
        DeviceIoControl(handle_,
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
    if (state_ != State::READY) {
        assert(false);
        setError(std::errc::resource_unavailable_try_again);
        return false;
    }
    if ((op_ & Op::READ_WRITE) == 0 || size_ == 0) {
        setSuccess();
        return false;
    }

    // initialize overlapped
    memset(&overlapped_, 0, sizeof(OVERLAPPED));

    steps_ = int(op_ & Op::READ_WRITE);

    // get data and size to read/write
    int result;
    if ((op_ & Op::WRITE) == 0) {
        // read
        result = ReadFile(device_.handle_, data_, capacity_, nullptr, &overlapped_);
    } else {
        // write
        result = WriteFile(device_.handle_, data_, size_, nullptr, &overlapped_);
    }

    if (!result) {
        int error = GetLastError();
        if (error != ERROR_IO_PENDING) {
            // error
            // ERROR_ACCESS_DENIED, ERROR_INVALID_HANDLE: device removed
            setSystemError(error);
            if (error == ERROR_ACCESS_DENIED || error == ERROR_INVALID_HANDLE)
                device_.close();
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

    if (steps_ != 0) {
        auto result = CancelIoEx(device_.handle_, &overlapped_);
        if (!result) {
            int error = GetLastError();
            setSystemError(error);
            //std::cerr << "cancel error " << e << std::endl;
            return false;
        }

        // clear pending read/write operations
        steps_ = 0;
    }
    return true;
}

void Uart_Win32::Buffer::onCompletion(OVERLAPPED *overlapped) {
    DWORD transferred;
    auto result = GetOverlappedResult(device_.handle_, overlapped, &transferred, false);
    if (result) {
        // success
        if (steps_ == int(Op::READ_WRITE)) {
            // read after write
            steps_ = int(Op::READ);

            // initialize overlapped
            memset(&overlapped_, 0, sizeof(OVERLAPPED));

            // read
            result = ReadFile(device_.handle_, data_, capacity_, nullptr, &overlapped_);
            if (!result) {
                int error = GetLastError();
                if (error != ERROR_IO_PENDING) {
                    // error
                    setSystemError(error);
                } else {
                    // -> onCompletion()
                    return;
                }
            } else {
                // -> onCompletion()
                return;
            }
        } else {
            setSuccess(transferred);
        }
    } else {
        // error
        // ERROR_ACCESS_DENIED, ERROR_INVALID_HANDLE: device removed
        // ERROR_OPERATION_ABORTED: cancelled
        auto error = GetLastError();
        setSystemError(error);
        if (error == ERROR_ACCESS_DENIED || error == ERROR_INVALID_HANDLE)
            device_.close();
    }

    // transfer finished
    setReady();
}

} // namespace coco
