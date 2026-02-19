#include <coco/platform/Loop_Win32.hpp> // includes Windows.h
#include <coco/Uart.hpp> // #undef PARITY_xxx


namespace coco {

/// @brief UART implementation using io_uring on Windows.
///
class Uart_io_uring : public Uart {
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
    class Buffer : public coco::Buffer, public IntrusiveListNode {
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
    Loop_io_uring &loop_;
    int baudRate_ = 0;

    // file handle
    static constexpr int INVALID_HANDLE_VALUE = -1;
    int file_ = INVALID_HANDLE_VALUE;

    // list of buffers
    IntrusiveList<Buffer> buffers_;
};


} // namespace coco
