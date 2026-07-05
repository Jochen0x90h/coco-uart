#include <Rs485Send-Test.hpp>
#include <coco/Loop.hpp>
#include <coco/debug.hpp>
#include <coco/BufferWriter.hpp>
#ifdef NATIVE
#include <iostream>
#endif


/*
    This test periodically sends "Hello RS485".
*/

using namespace coco;

Coroutine send(Loop &loop, Uart &rs485, Buffer &buffer) {
    while (true) {
        // wait until port is ready
        debug::out << "Wait for serial port...\n";
        co_await rs485.untilReady();

        while (buffer.ready()) {
            debug::toggleGreen();
            co_await buffer.write("Hello RS485");
            co_await loop.sleep(50ms);
        }
    }
}

#ifdef NATIVE
// Windows/Linux/MacOS: Pass serial ports device as arguments, e.g. "COM10", "COM11" or "ttyUSB0", "ttyUSB1"
int main(int argc, char **argv) {
    if (argc < 2)
        return 1;

    // add listener that opens the uart given as command line argument as soon as it appears
    drivers.monitor.listenAdd([&](const std::filesystem::path &path, String name) {
        debug::out << name << " (" << path.string() << ")\n";
        if (name == argv[1])
            drivers.uart.open(path, Uart::Format::DEFAULT, 38400, 20ms);
    });
#else
int main() {
#endif
    send(drivers.loop, drivers.rs485, drivers.sendBuffer);

    drivers.loop.run();

    return 0;
}
