#include <UartSendTest.hpp>
#include <coco/Loop.hpp>
#include <coco/debug.hpp>
#include <coco/BufferWriter.hpp>
#ifdef NATIVE
#include <iostream>
#endif


/*
    This test periodically sends "Hello UART". It can be tested on a serial port with loopback.
    For a RS-232 port with 9-pin D-Sub connector, connect pins 2 and 3.
    On embedded platforms, connect TX to RX.
    The send() coroutine sends "Hello UART" every half second, outputs "Send" to the debug output and toggles the blue LED.
    The recieve() coroutine outputs the received string to the debug output and toggles the green LED if "Hello UART"
    was received. On Error (timeout or received something other than "Hello UART"), the red LED gets toggled.
*/

using namespace coco;


// periodically send "Hello UART"
Coroutine send(Loop &loop, Uart &uart, Buffer &buffer) {
    bool toggle = false;
    while (buffer.ready()) {
        //uart.setControl(Uart::ControlLineState::DTR | Uart::Control::RTS);

        debug::out << "Send\n";
#ifndef NATIVE
        debug::toggleBlue();
#endif
        co_await buffer.write("Hello UART");
        co_await loop.sleep(500ms);

        // make sure no read/write operation is in progress when calling setBaudRate()
        //uart.setBaudRate(toggle ? 38400 : 100000);
        toggle = !toggle;
    }

    // failed to open device or device stopped working
    loop.exit();
}

// receive the "HEllo UART" from send()
Coroutine receive(Loop &loop, Buffer &buffer) {
    while (buffer.ready()) {
        buffer.startRead();
        //int r = co_await select(buffer.untilReadyOrDisabled(), loop.sleep(2s));
        int r = 1; co_await buffer.untilReadyOrDisabled();
        if (r == 1) {
            debug::out << buffer.string() << '\n';
#ifndef NATIVE
            if (buffer.string() == "Hello UART") {
                // ok
                debug::toggleGreen();
            } else {
                // error
                debug::toggleRed();
            }
#endif
        } else {
            // timeout
            debug::out << "Error: Timeout\n";
#ifndef NATIVE
            debug::toggleRed();
#endif
            buffer.cancel();
            co_await buffer.untilReadyOrDisabled();
        }
    }
    debug::set(debug::MAGENTA);

    // failed to open device or device stopped working
    loop.exit();
}

// detect change of serial state (DCD, DSR, RI)
Coroutine detect(Loop &loop, Uart &uart) {
    while (true) {
        co_await uart.untilSignalsChanged();
        auto signals = uart.getInputSignals();

        debug::out << "Serial State:";
        if ((signals & Uart::InputSignals::DCD) != 0)
            debug::out << " DCD";
        if ((signals & Uart::InputSignals::DSR) != 0)
            debug::out << " DSR";
        if ((signals & Uart::InputSignals::RI) != 0)
            debug::out << " RI";
        if ((signals & Uart::InputSignals::CTS) != 0)
            debug::out << " CTS";
        debug::out << '\n';
    }
}

#ifdef NATIVE
// Windows/Linux/MacOS: Pass serial port device as argument, e.g. "\\\\.\\COM9"
int main(int argc, char **argv) {
    if (argc < 2)
        return 1;
    drivers.init(argv[1]);
#else
int main() {
#endif
    send(drivers.loop, drivers.uart, drivers.sendBuffer);
    receive(drivers.loop, drivers.receiveBuffer);
    detect(drivers.loop, drivers.uart);

    drivers.loop.run();

    return 0;
}
