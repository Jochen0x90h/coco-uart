#pragma once

#include <coco/platform/Loop_native.hpp> // includes Windows.h, #undef PARITY_xxx
#include <coco/Uart.hpp>


namespace coco {

/// @brief UART implementation using IO completion ports on Windows.
/// The COM-Port can appear and disappear e.g. when it is a USB device. The device goes to Device::State::DISABLED
/// when the COM-Port disappears. When open() is called, the device immediately goes to Device::State::OPENING and
/// eventually to Device::State::READY when the COM-Port appears again.
class Uart_Win32 : public Uart, public Loop_Win32::TimeoutHandler, public Loop_Win32::CompletionHandler {
public:
    /// @brief Constructor/
    /// @param loop event loop
    /// @param baudRate baud rate (e.g. 38400)
    /// @param format frame format
    /// @param rxTimeout receiver timeout in milliseconds, at least ~20ms (note that setRxTimeout() is in bit times)
    Uart_Win32(Loop_Win32 &loop, Format format, int baudRate, Milliseconds<> rxTimeout)
        : Uart(State::DISABLED)
        , loop_(loop)
        , format_(format), baudRate_(baudRate), rxTimeout_(rxTimeout) {}

    ~Uart_Win32() override;

    // Uart methods
    void setPath(const std::filesystem::path &path) override;
    using Uart::setPath;
    bool open() override;
    void setValue(int id, int value) override;
    int getValue(int id) override;

    // BufferDevice methods
    class Buffer;
    int getBufferCount() override;
    Buffer &getBuffer(int index) override;

    // Device methods
    void close() override;


    /// @brief Buffer for transferring data to/from a UART device (COM port)
    ///
    class Buffer : public coco::Buffer, public coco::IntrusiveListNode {
        friend class Uart_Win32;
    public:
        Buffer(Uart_Win32 &device, int size);
        ~Buffer() override;

        // Device methods
        bool start() override;
        bool cancel() override;

    protected:
        void onCompletion(OVERLAPPED *overlapped);

        Uart_Win32 &device_;
        OVERLAPPED overlapped_;
    };

protected:
    void onTimeout() override;
    void onCompletion(OVERLAPPED *overlapped) override;

    Loop_Win32 &loop_;
    Format format_;
    int baudRate_;
    Milliseconds<> rxTimeout_;
    std::filesystem::path path_;

    // file handle
    HANDLE file_ = INVALID_HANDLE_VALUE;

    // overlapped for monitoring events (e.g. change of DSR signal)
    OVERLAPPED overlapped_;
    ULONG mask_;

    // list of buffers
    IntrusiveList<Buffer> buffers_;
};


} // namespace coco
