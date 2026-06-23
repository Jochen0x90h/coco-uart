#include <UartTest.hpp>
#include <coco/Coroutine.hpp>
#include <coco/Loop.hpp>
#include <coco/debug.hpp>
#include <coco/BufferWriter.hpp>
#include <coco/StreamOperators.hpp>
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
    int baudRate = 38400;
    while (true) {
        // open the uart and wait until ready
        uart.open();
        debug::out << "Wait for serial port...\n";
        co_await uart.untilReadyOrDisabled();

        while (buffer.ready()) {
            //uart.setOutputSignals(Uart::OutputSignals::DTR | Uart::OutputSignals::RTS);

            debug::out << "Send\n";
            debug::toggleBlue();

            co_await buffer.write("Hello UART");
            co_await loop.sleep(500ms);

            // toggle baud rate
            uart.setBaudRate(baudRate);
            baudRate ^= 100000 ^ 38400;
        }
    }

    // failed to open device or device stopped working
    //loop.exit();
}

// receive the "Hello UART" from send()
Coroutine receive(Loop &loop, Uart &uart, Buffer &buffer) {
    while (true) {
        // wait until ready
        co_await uart.untilReady();

        while (buffer.ready()) {
            // overwrite entire buffer
            buffer.resize(buffer.capacity()).array<char>().fill('x');

            // start receiving
            debug::out << "Receive\n";
            buffer.startRead();
            int r = co_await select(buffer.untilReadyOrDisabled(), loop.sleep(2s));
            if (r == 1) {
                // output received string to debug console
                debug::out << "Received " << buffer.string();

                if (buffer.string() == "Hello UART") {
                    // ok
                    debug::toggleGreen();
                    debug::clearRed();
                } else {
                    // error
                    debug::toggleRed();
                    debug::out << " (error: size " << dec(buffer.size()) << ")\n";
                }
                debug::out << '\n';
            } else {
                // timeout
                debug::out << "Error: Timeout\n";
                debug::toggleRed();
                buffer.cancel();
                co_await buffer.untilReadyOrDisabled();
            }

        }
    }

    // failed to open device or device stopped working
    //debug::set(debug::MAGENTA);
    //loop.exit();
}

// detect change of serial state (DCD, DSR, RI)
Coroutine state(Loop &loop, Uart &uart) {
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
// Windows/Linux/MacOS: Pass serial port device as argument, e.g. "COM10" or "ttyUSB0"
int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "Error: No device specified" << std::endl;
        return 1;
    }
    drivers.uart.setPath(argv[1]);
#else
int main() {
#endif
    debug::out << "UartTest\n";

    //receive(drivers.loop, drivers.uart, drivers.receiveBuffer);
    send(drivers.loop, drivers.uart, drivers.sendBuffer);
    state(drivers.loop, drivers.uart);

    drivers.loop.run();

    return 0;
}
