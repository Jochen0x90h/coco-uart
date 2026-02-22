#include <coco/platform/Loop_native.hpp>
#include <coco/Uart.hpp>


namespace coco {

/// @brief UART implementation using io_uring on Windows.
///
class Uart_io_uring : public Uart, public Loop_io_uring::CompletionHandler {
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

    /// @brief Open device by name.
    /// Fails if already open. Calling close() is ok if the uart is not open.
    /// @param name device name
    /// @return ture if successful
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
    class Buffer : public coco::Buffer, public Loop_io_uring::CompletionHandler, public IntrusiveListNode {
        friend class Uart_io_uring;
    public:
        Buffer(Uart_io_uring &device, int size);
        ~Buffer() override;

        // Device methods
        bool start() override;
        bool cancel() override;

    protected:
        void handle(io_uring_cqe &cqe);

        Uart_io_uring &device_;
    };

protected:
    void handle(io_uring_cqe &cqe);

    Loop_io_uring &loop_;
    int baudRate_ = 0;

    // file handle
    static constexpr int INVALID_HANDLE_VALUE = -1;
    int com_ = INVALID_HANDLE_VALUE;

    // list of buffers
    IntrusiveList<Buffer> buffers_;

    // list of active transfers
    //InterruptQueue<Buffer> receiveTransfers_;
    //InterruptQueue<Buffer> sendTransfers_;

    //int poll_ = 0;
};


} // namespace coco
