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

/// @brief Asynchronous receiver/transmitter (UART) abstraction.
/// Supports adjustment of baud rate, parity etc.
class Uart : public BufferDevice {
public:
    /// @brief Value IDs.
    /// An implementation may support only a subset of the values.
    /// struct Value only serves as a namespace
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

    /// @brief Frame format
    ///
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

    /// @brief Modem control line state
    /// Note: Is similar to Control Line State of CDC PSTN subclass (6.3.12, Table 18, https://www.usb.org/document-library/class-definitions-communication-devices-12).
    /// Therefore similar to usb::PstnControlLineState of coco-usb
    enum class OutputSignals {
        NONE = 0,

        // Data Terminal Ready
        DTR = 1,

        // Request To Send
        RTS = 1 << 1,
    };

    /// @brief Serial State
    /// Note: Is similar to Serial State of CDC PSTN subclass (6.5.4, Table 31, https://www.usb.org/document-library/class-definitions-communication-devices-12).
    /// Therefore similar to usb::cdc::PstnSerialState of coco-usb except for CTS
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

    /// @brief Set a configuration or state value.
    /// @param id id of value to change, either pre-defined or implementation specific
    /// @param value value to set
    virtual void setValue(int id, int value) = 0;

    /// @brief Set the current baud rate
    ///
    void setBaudRate(int baudRate) {setValue(Value::BAUD, baudRate);}

    /// @brief Set the current frame format
    ///
    void setFormat(Format format) {setValue(Value::FORMAT, int(format));}

    /// @brief Set the current RX timeout in bit times
    ///
    void setRxTimeout(int timeout) {setValue(Value::RX_TIMEOUT, timeout);}

    /// @brief Set the output signals
    ///
    void setOutputSignals(OutputSignals signals) {setValue(Value::OUTPUT_SIGNALS, int(signals));}

    /// @brief Set the communication channel index
    ///
    void setChannel(int index) {setValue(Value::CHANNEL, index);}

    /// @brief Get a configuration or state value.
    /// @param id id of value to get, either pre-defined or implementation specific
    /// @returns value for given id
    virtual int getValue(int id) = 0;

    /// @brief Get the current baud rate
    ///
    int getBaudRate() {return getValue(Value::BAUD);}

    /// @brief Get the current frame format
    ///
    Format getFormat() {return Format(getValue(Value::FORMAT));}

    /// @brief Get the state of the input signals. Use co_await uart.untilNewSignals(); to wait for a state change
    ///
    InputSignals getInputSignals() {return InputSignals(getValue(Value::INPUT_SIGNALS));}

    /// @brief Wait until input control signals changed (e.g. InputSignals::DSR or InputSignals::RI)
    /// @return use co_await on return value to wait until the input control signals change
    [[nodiscard]] Awaitable<Events> untilSignalsChanged() {
        return {this->st.tasks, Events::SIGNALS_CHANGED};
    }


    using Buffer = coco::Buffer;
};
COCO_ENUM(Uart::Format)
COCO_ENUM(Uart::OutputSignals)
COCO_ENUM(Uart::InputSignals)

} // namespace coco
