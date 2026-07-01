#pragma once

#include <coco/Uart.hpp>
#include <coco/InterruptQueue.hpp>
#include <coco/platform/Loop_native.hpp>


namespace coco {

/// @brief UART implementation using io_uring on Windows.
///
class Uart_io_uring : public Uart, public Loop_io_uring::CompletionHandler, public Loop_io_uring::TimeoutHandler {
public:
    /// @brief Constructor/
    /// @param loop event loop
    /// @param baudRate baud rate (e.g. 38400)
    /// @param format frame format
    /// @param rxTimeout receiver timeout in milliseconds, at least ~20ms (note that setRxTimeout() is in bit times)
    Uart_io_uring(Loop_io_uring &loop)
        : Uart(State::DISABLED)
        , loop_(loop) {}

    ~Uart_io_uring() override;

    /// @brief Open device by path.
    /// Fails if already open. Calling close() does nothing if the uart is not open.
    /// @param path Device path (e.g. "/dev/ttyUSB0")
    /// @param format Data format (number of data and stop bits)
    /// @param baudRate Baud Rate
    /// @param rxTimeout Receive timeout (min. 20ms)
    /// @return true if successful
    bool open(const std::filesystem::path &path, Format format, int baudRate, Milliseconds<> rxTimeout);

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
    class Buffer : public coco::Buffer, public Loop_io_uring::CompletionHandler, public coco::IntrusiveListNode,
        public IntrusiveMpscQueueNode
    {
        friend class Uart_io_uring;
    public:
        Buffer(Uart_io_uring &device, int size);
        ~Buffer() override;

        // Device methods
        bool start() override;
        bool cancel() override;

    protected:
        void onCompletion(io_uring_cqe &cqe, int id) override;

        Uart_io_uring &device_;
    };

protected:
    void onCompletion(io_uring_cqe &cqe, int id) override;
    void onTimeout();

    Loop_io_uring &loop_;
    int baudRate_ = 0;
    Milliseconds<> rxTimeout_;

    // file handle
    static constexpr int INVALID_HANDLE_VALUE = -1;
    int com_ = INVALID_HANDLE_VALUE;

    // list of buffers
    IntrusiveList<Buffer> buffers_;

    // list of active transfers
    InterruptQueue<Buffer> receiveTransfers_;

    // receive buffer
    uint8_t receiveBuffer[32];
    int receivedSize_ = 0;
};


} // namespace coco
