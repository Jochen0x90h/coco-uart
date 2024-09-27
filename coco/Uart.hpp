#pragma once

#include <coco/BufferDevice.hpp>
#include <coco/Frequency.hpp>
#include <coco/String.hpp>
#include <cstdint>


#undef PARITY_NONE
#undef PARITY_ODD
#undef PARITY_EVEN
#undef PARITY_MARK
#undef PARITY_SPACE

namespace coco {

/**
 * Asynchronous receiver/transmitter (UART) abstraction with support for setConfig() method to adjust baud rate,
 * parity etc. on the fly.
 * Microcontroller implementations can directly inherit from BufferDevice if configure() is not supported.
 */
class Uart : public BufferDevice {
public:

    /**
     * Value IDs. An implementation may support only a subset of the values.
     * struct Value only serves as a namespace
     */
    struct Value {
        // frame format configuration id
        static constexpr int FORMAT = 0;

        // baud rate configuration id
        static constexpr int BAUD = 1;

        // receiver timeout in bit times. Note that changing the baud rate may not update the timeout
        static constexpr int RX_TIMEOUT = 2;

        // modem control output signals (DTR and RTS)
        static constexpr int OUTPUT_SIGNALS = 3;

        // modem control input signals (DSR, DCD, RING_SIGNAL)
        static constexpr int INPUT_SIGNALS = 4;

        // communication channel index
        static constexpr int CHANNEL = 5;
    };

    /**
     * Frame format
     */
    enum class Format {
        // number of data bits
        DATA_5 = 5,
        DATA_6 = 6,
        DATA_7 = 7,
        DATA_8 = 8,
        DATA_9 = 9,
        DATA_16 = 16,
        DATA_MASK = 0x1f,

        // parity
        PARITY_NONE = 0,
        PARITY_ODD = 1 << 5,
        PARITY_EVEN = 2 << 5,
        PARITY_MARK = 3 << 5,
        PARITY_SPACE = 4 << 5,
        PARITY_MASK = 7 << 5,

        // number of stop bits
        STOP_1 = 0,
        STOP_1_5 = 1 << 8,
        STOP_2 = 2 << 8,
        STOP_MASK = 3 << 8,

        // default format
        DEFAULT = DATA_8 | PARITY_NONE | STOP_1,
    };

    /**
     * Modem control line state (is same as usb::PstnControlLineState)
     */
    enum class OutputSignals {
        NONE = 0,

        // Data Terminal Ready
        DTR = 1,

        // Request To Send
        RTS = 1 << 1,
    };

    /**
     * Serial State (is same as usb::PstnSerialState except for CTS)
     */
    enum class InputSignals {
        NONE = 0,

        // Data Carrier Detect
        DCD = 1,

        // Data Set Ready
        DSR = 1 << 1,

        // Ring Indicator
        RI = 1 << 3,

        // Clear To Send
        CTS = 1 << 7,

        BREAK = 1 << 2,
        FRAMING_ERROR = 1 << 4,
        PARITY_ERROR = 1 << 5,
        OVERRUN_ERROR = 1 << 6
    };


    Uart(State state) : BufferDevice(state) {}

    /**
     * Set a configuration or state value.
     * @param id id of value to change, either pre-defined or implementation specific
     * @param value value to set
     */
    virtual void setValue(int id, int value) = 0;

    /// Set the current baud rate
    void setBaudRate(int baudRate) {setValue(Value::BAUD, baudRate);}

    /// Set the current frame format
    void setFormat(Format format) {setValue(Value::FORMAT, int(format));}

    /// Set the current RX timeout in bit times
    void setRxTimeout(int timeout) {setValue(Value::RX_TIMEOUT, timeout);}

    /// Set the output signals
    void setOutputSignals(OutputSignals signals) {setValue(Value::OUTPUT_SIGNALS, int(signals));}

    /// Set the communication channel index
    void setChannel(int index) {setValue(Value::CHANNEL, index);}

    /**
     * Get a configuration or state value.
     * @param id id of value to get, either pre-defined or implementation specific
     * @returns value for given id
     */
    virtual int getValue(int id) = 0;

    /// Get the current baud rate
    int getBaudRate() {return getValue(Value::BAUD);}

    /// Get the current frame format
    Format getFormat() {return Format(getValue(Value::FORMAT));}

    /// Get the state of the input signals. Use co_await uart.untilNewSignals(); to wait for a state change
    InputSignals getInputSignals() {return InputSignals(getValue(Value::INPUT_SIGNALS));}

    /**
     * Wait until control signals changed (e.g. DCD for serial or volgate level for USB PD)
     * @return use co_await on return value to wait until the control signals change
     */
    [[nodiscard]] Awaitable<Events> untilSignalsChanged() {
        //auto &st = getStateTasks();
        return {this->st.tasks, Events::SIGNALS_CHANGED};
    }


    using Buffer = coco::Buffer;
};
COCO_ENUM(Uart::Format)
COCO_ENUM(Uart::OutputSignals)
COCO_ENUM(Uart::InputSignals)

} // namespace coco
