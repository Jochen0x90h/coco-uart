#define NOMINMAX
#include <Windows.h>
#include <ioapiset.h>
#include <ntddser.h>

#include "Uart_Win32.hpp"
#include <coco/bits.hpp>
#include <iostream>
#include <filesystem>


namespace coco {

Uart_Win32::~Uart_Win32() {
    CloseHandle(this->file);
}

//StateTasks<const Device::State, Device::Events> &Uart_Win32::getStateTasks() {
//	return makeConst(this->st);
//}

// todo: test what happens when buffers are busy when we call close()
void Uart_Win32::close() {
    // close file
    CloseHandle(this->file);
    this->file = INVALID_HANDLE_VALUE;

    // set state
    this->st.state = State::CLOSING;

    // disable buffers
    for (auto &buffer : this->buffers) {
        buffer.setDisabled();
    }

    // set state
    this->st.state = State::DISABLED;

    // resume all coroutines waiting for state change
    this->st.doAll(Events::ENTER_CLOSING | Events::ENTER_DISABLED);
}

int Uart_Win32::getBufferCount() {
    return this->buffers.count();
}

Uart_Win32::Buffer &Uart_Win32::getBuffer(int index) {
    return this->buffers.get(index);
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
            DCB dcb;
            dcb.DCBlength = sizeof(dcb);
            GetCommState(this->file, &dcb);
            int dataBits = extract(value, int(Format::DATA_MASK));
            int parity = extract(value, int(Format::PARITY_MASK));
            int stopBits = extract(value, int(Format::STOP_MASK));
            dcb.ByteSize = dataBits;
            dcb.fParity = parity == 0 ? 0 : 1;
            dcb.Parity = parity;
            dcb.StopBits = stopBits;
            dcb.fDtrControl = 0;
            dcb.fRtsControl = 0;
            SetCommState(this->file, &dcb);
        }
        break;
    case Value::BAUD:
        {
            this->baudRate = value;
            DCB dcb;
            dcb.DCBlength = sizeof(dcb);
            GetCommState(this->file, &dcb);
            dcb.BaudRate = value;
            SetCommState(this->file, &dcb);
        }
        break;
    case Value::RX_TIMEOUT:
        {
            int rxTimeout = std::max(value * 1000 / this->baudRate + 1, 20);

            // https://learn.microsoft.com/en-us/windows/win32/api/winbase/nf-winbase-setcommtimeouts
            COMMTIMEOUTS timeouts;
            timeouts.ReadIntervalTimeout = rxTimeout;
            timeouts.ReadTotalTimeoutMultiplier = 0;
            timeouts.ReadTotalTimeoutConstant = 0;
            timeouts.WriteTotalTimeoutMultiplier = 0;
            timeouts.WriteTotalTimeoutConstant = 0;
            SetCommTimeouts(this->file, &timeouts);
        }
        break;
    case Value::OUTPUT_SIGNALS:
        // http://www.ioctls.net/
        {
            bool dtr = (OutputSignals(value) & OutputSignals::DTR) != 0;
            bool rts = (OutputSignals(value) & OutputSignals::RTS) != 0;
            DWORD transferred;

            // RTS needs to be send first because of a bug in usbser.sys (https://answers.microsoft.com/en-us/windows/forum/all/usbsersys-does-not-handle-rts-signal-correctly/e348047e-dacd-47d5-8e74-1fd2f275bebb)
            DeviceIoControl(this->file,
                rts ? IOCTL_SERIAL_SET_RTS : IOCTL_SERIAL_CLR_RTS,
                nullptr, 0, // input buffer
                nullptr, 0, // output buffer
                &transferred,
                nullptr);
            DeviceIoControl(this->file,
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
            DeviceIoControl(this->file, IOCTL_SERIAL_GET_MODEMSTATUS,
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

bool Uart_Win32::open(String name, Format format, int baudRate, Milliseconds<> rxTimeout) {
    // open file using Win32
    std::filesystem::path path(std::u8string_view(reinterpret_cast<const char8_t *>(name.data()), name.size()));
    HANDLE file = CreateFileW(path.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0, // no sharing
        nullptr, // security
        OPEN_EXISTING, // open existing port
        FILE_FLAG_OVERLAPPED,
        nullptr);
    if (file == INVALID_HANDLE_VALUE) {
        //int e = WSAGetLastError();
        return false;
    }

    // add file to completion port of event loop
    Loop_Win32::CompletionHandler *handler = this;
    if (CreateIoCompletionPort(
        file,
        loop.port,
        ULONG_PTR(handler),
        0) == nullptr)
    {
        //int e = WSAGetLastError();
        CloseHandle(file);
        return false;
    }
    this->file = file;

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
    SetCommTimeouts(this->file, &timeouts);


    // set state
    this->st.state = State::OPENING;

    // enable buffers
    for (auto &buffer : this->buffers) {
        buffer.setReady(0);
    }

    // set state
    this->st.state = State::READY;

    // resume all coroutines waiting for state change
    this->st.doAll(Events::ENTER_OPENING | Events::ENTER_READY);

    // set wait mask
    ULONG value = SERIAL_EV_RLSD | SERIAL_EV_DSR | SERIAL_EV_RING | SERIAL_EV_CTS;
    DWORD transferred;
    auto result = DeviceIoControl(this->file,
        IOCTL_SERIAL_SET_WAIT_MASK,
        &value, 4, // input buffer
        nullptr, 0, // output buffer
        nullptr, // transfered into output buffer
        nullptr);

    // initialize overlapped
    memset(&this->overlapped, 0, sizeof(OVERLAPPED));

    // wait for events
    DeviceIoControl(this->file,
        IOCTL_SERIAL_WAIT_ON_MASK,
        nullptr, 0, // input buffer
        &this->mask, 4, // output buffer
        nullptr, // transfered into output buffer
        &this->overlapped);
    return true;
}

void Uart_Win32::handle(OVERLAPPED *overlapped) {
    for (auto &buffer : this->buffers) {
        if (overlapped == &buffer.overlapped) {
            buffer.handle(overlapped);
            return;
        }
    }
    if (overlapped = &this->overlapped) {
        DWORD transferred;
        auto result = GetOverlappedResult(this->file, overlapped, &transferred, false);
        if (!result) {
            auto error = GetLastError();
            int x = 0;
        } else {
            // resume all coroutines waiting for state change
            this->st.doAll(Events::SIGNALS_CHANGED);
        }

        // wait for events again
        DeviceIoControl(this->file,
            IOCTL_SERIAL_WAIT_ON_MASK,
            nullptr, 0, // input buffer
            &this->mask, 4, // output buffer
            nullptr, // transfered into output buffer
            &this->overlapped);
    }
}


// Buffer

Uart_Win32::Buffer::Buffer(Uart_Win32 &device, int size)
    : coco::Buffer(new uint8_t[size], size, device.st.state)
    , device(device)
{
    device.buffers.add(*this);
}

Uart_Win32::Buffer::~Buffer() {
    delete [] this->p.data;
}

bool Uart_Win32::Buffer::start(Op op) {
    if (this->st.state != State::READY) {
        assert(this->st.state != State::BUSY);
        return false;
    }

    // check if READ or WRITE flag is set
    assert((op & Op::READ_WRITE) != 0);

    // initialize overlapped
    memset(&this->overlapped, 0, sizeof(OVERLAPPED));

    // get data and size to read/write
    int result;
    if ((op & Op::WRITE) == 0) {
        // read
        this->op = Op::NONE;
        result = ReadFile(this->device.file, this->p.data, this->p.capacity, nullptr, &this->overlapped);
    } else {
        // write
        this->op = op;
        result = WriteFile(this->device.file, this->p.data, this->p.size, nullptr, &this->overlapped);
    }

    if (result == 0) {
        int error = GetLastError();
        if (error != ERROR_IO_PENDING) {
            // "real" error
            setReady(0);
            return false;
        }
    }

    // set state
    setBusy();

    return true;
}

bool Uart_Win32::Buffer::cancel() {
    if (this->st.state != State::BUSY)
        return false;

    auto result = CancelIoEx(this->device.file, &this->overlapped);
    if (!result) {
        auto e = GetLastError();
        std::cerr << "cancel error " << e << std::endl;
    }
    return true;
}

void Uart_Win32::Buffer::handle(OVERLAPPED *overlapped) {
    DWORD transferred;
    auto result = GetOverlappedResult(this->device.file, overlapped, &transferred, false);
    if (!result) {
        // "real" error or cancelled (ERROR_OPERATION_ABORTED): return zero size
        auto error = GetLastError();
        //transferred = 0;

        // transfer finished
        setReady(0);
    } else {
        // transfer OK
        if ((this->op & Op::READ) != 0) {
            // read after write
            this->op = Op::NONE;
            result = ReadFile(this->device.file, this->p.data, this->p.capacity, nullptr, &this->overlapped);
        } else {
            // transfer finished
            setReady(transferred);
        }
    }
}

} // namespace coco
