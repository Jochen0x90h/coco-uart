#include <coco/Uart.hpp>
#include <coco/platform/Loop_Win32.hpp> // includes Windows.h


namespace coco {

/**
 * Implementation of Uart interface on Win32 using IO completion ports
 */
class Uart_Win32 : public Uart, public Loop_Win32::CompletionHandler {
public:
    /**
     * Constructor
     * @param loop event loop
     * @param baudRate baud rate (e.g. 38400)
     * @param format frame format
     * @param rxTimeout receiver timeout in milliseconds, at least ~20ms (note that setRxTimeout() is in bit times)
     */
    Uart_Win32(Loop_Win32 &loop)
        : Uart(State::DISABLED)
        , loop(loop) {}

    ~Uart_Win32() override;

    class Buffer;

    // Device methods
    //StateTasks<const State, Events> &getStateTasks() override;
    void close() override;

    // BufferDevice methods
    int getBufferCount() override;
    Buffer &getBuffer(int index) override;

    // Uart methods
    void setValue(int id, int value) override;
    int getValue(int id) override;

    /**
     * Open device by name
     * @param name device name
     */
    bool open(String name, Format format, int baudRate, Milliseconds<> rxTimeout);


    /**
     * Buffer for transferring data to/from a UART device (COM port)
     */
    class Buffer : public coco::Buffer, public IntrusiveListNode {
        friend class Uart_Win32;
    public:
        Buffer(Uart_Win32 &device, int size);
        ~Buffer() override;

        // Device methods
        bool start(Op op) override;
        bool cancel() override;

    protected:
        void handle(OVERLAPPED *overlapped);

        Uart_Win32 &device;

        Op op;
        OVERLAPPED overlapped;
    };

protected:
    void handle(OVERLAPPED *overlapped) override;

    Loop_Win32 &loop;
    int baudRate = 0;

    // file handle
    HANDLE file = INVALID_HANDLE_VALUE;

    // overlapped for monitoring events (e.g. change of DSR signal)
    OVERLAPPED overlapped;
    ULONG mask;

    // device state
    //StateTasks<State, Events> st = State::DISABLED;

    // list of buffers
    IntrusiveList<Buffer> buffers;
};


} // namespace coco
