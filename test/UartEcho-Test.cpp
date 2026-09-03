#include <Uart-Test.hpp>
#include <coco/Loop.hpp>
#include <coco/debug.hpp>
#include <coco/BufferWriter.hpp>
#ifdef NATIVE
#include <iostream>
#endif


/*
    This test echos all received characters back to the sender.
*/

using namespace coco;


Coroutine echo(Loop &loop, Uart &uart, Buffer &buffer) {
    while (true) {
        // wait until port is ready
        debug::out << "Wait for serial port...\n";
        co_await uart.untilReady();

        while (buffer.ready()) {
            // receive something
            co_await buffer.read();
            int transferred = buffer.size();

#ifdef NATIVE
            std::cout << buffer.string() << std::endl;
#else
            debug::toggleGreen();
#endif
            //co_await loop.sleep(500ms);

            // send it back
            co_await buffer.write(transferred);
        }
    }
}

#ifdef NATIVE
// Windows/Linux/MacOS: Pass serial ports device as arguments, e.g. "COM10", "COM11" or "ttyUSB0", "ttyUSB1"
int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "Error: No device specified" << std::endl;
        return 1;
    }

    // add listener that opens the uart given as command line argument as soon as it appears
    drivers.monitor.listenAdd([&](DevicePath path, String name) {
        debug::out << name << " (" << path << ")\n";
        if (name == argv[1])
            drivers.uart.open(path, Uart::Format::DEFAULT, 38400, 20ms);
    });
#else
int main() {
#endif
    echo(drivers.loop, drivers.uart, drivers.sendBuffer);

    drivers.loop.run();
    return 0;
}
