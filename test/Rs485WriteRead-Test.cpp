#include <Rs485WriteRead-Test.hpp>
#include <coco/Loop.hpp>
#include <coco/debug.hpp>
#include <coco/BufferWriter.hpp>
#ifdef NATIVE
#include <iostream>
#endif


/*
    This test periodically sends "Hello UART" and waits for a reply. Needs two connected serial ports.
    For two RS485 ports, simply connect A and B of both ports.
    On embedded platforms, connect TX 1 to RX 2 and TX 2 to RX 1. The green LED toggles every half second if everything is ok.
*/

using namespace coco;


Coroutine writeRead(Loop &loop, Uart &uart, Buffer &buffer) {
    while (true) {
        // wait until port is ready
        debug::out << "Wait for serial port...\n";
        co_await uart.untilReady();

        while (buffer.ready()) {
            // write and immediately read reply
            co_await buffer.write("Hello UART", Buffer::Op::READ);

            // wait for up to 2 seconds for a reply
            int r = co_await select(buffer.untilReadyOrDisabled(), loop.sleep(2s));

            if (r == 1) {
                if (buffer.string() == "Hello UART") {
                    // OK
                    debug::toggleGreen();
                } else {
                    // error: wrong text received
                    debug::toggleBlue();
                }
                co_await loop.sleep(500ms);
            } else {
                // error: timeout
                debug::toggleRed();
            }
        }
    }
}

Coroutine echo(Loop &loop, Uart &uart, Buffer &buffer) {
    while (true) {
        // wait until port is ready
        debug::out << "Wait for serial port...\n";
        co_await uart.untilReady();

        while (buffer.ready()) {
            // receive something
            co_await buffer.read();

            // send it back
            co_await buffer.write();
        }
    }
}

#ifdef NATIVE
// Windows/Linux/MacOS: Pass serial ports device as arguments, e.g. "COM10", "COM11" or "ttyUSB0", "ttyUSB1"
int main(int argc, char **argv) {
    if (argc < 3)
        return 1;

    // add listener that opens the uart given as command line argument as soon as it appears
    drivers.monitor.listenAdd([&](const std::filesystem::path &path, String name) {
        debug::out << name << " (" << path.string() << ")\n";
        if (name == argv[1])
            drivers.uart1.open(path, Uart::Format::DEFAULT, 38400, 20ms);
        if (name == argv[2])
            drivers.uart2.open(path, Uart::Format::DEFAULT, 38400, 20ms);
    });
#else
int main() {
#endif
    //debug::setRed();
    writeRead(drivers.loop, drivers.uart1, drivers.buffer1);
    echo(drivers.loop, drivers.uart2, drivers.buffer2);

    drivers.loop.run();

    return 0;
}
