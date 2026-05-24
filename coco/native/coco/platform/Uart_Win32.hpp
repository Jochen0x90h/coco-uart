#pragma once

#include <coco/platform/Loop_native.hpp> // includes Windows.h, #undef PARITY_xxx
#include <coco/Uart.hpp>


namespace coco {

/// @brief UART implementation using IO completion ports on Windows.
///
class Uart_Win32 : public Uart, public Loop_Win32::CompletionHandler {
public:
    /// @brief Constructor/
    /// @param loop event loop
    /// @param baudRate baud rate (e.g. 38400)
    /// @param format frame format
    /// @param rxTimeout receiver timeout in milliseconds, at least ~20ms (note that setRxTimeout() is in bit times)
    Uart_Win32(Loop_Win32 &loop)
        : Uart(State::DISABLED)
        , loop_(loop) {}

    ~Uart_Win32() override;

    /// @brief Open device by name.
    /// Fails if already open. Calling close() is ok if the uart is not open.
    /// @param name Device name (e.g. "\\\\.\\COM9")
    /// @param format Data format (number of data and stop bits)
    /// @param baudRate Baud Rate
    /// @param rxTimeout Receive timeout (min. 20ms)
    /// @return true if successful
    bool open(String name, Format format, int baudRate, Milliseconds<> rxTimeout);

    // Uart methods
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
    class Buffer : public coco::Buffer, public IntrusiveListNode {
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
    void onCompletion(OVERLAPPED *overlapped) override;

    Loop_Win32 &loop_;
    int baudRate_ = 0;

    // file handle
    HANDLE file_ = INVALID_HANDLE_VALUE;

    // overlapped for monitoring events (e.g. change of DSR signal)
    OVERLAPPED overlapped_;
    ULONG mask_;

    // list of buffers
    IntrusiveList<Buffer> buffers_;
};


} // namespace coco
