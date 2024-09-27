#include <Rs485WriteReadTest.hpp>
#include <coco/Loop.hpp>
#include <coco/debug.hpp>
#include <coco/BufferWriter.hpp>
#ifdef NATIVE
#include <iostream>
#endif


/*
    This test periodically sends "Hello UART" and waits for a reply. Needs two connected serial ports.
    For two RS485 ports, simply connect A and B of both ports.
    On embedded platforms, connect TX 1 to RX 2 and TX 2 to TX 1. The green LED toggles every half second if everything is ok.
*/

using namespace coco;


Coroutine writeRead(Loop &loop, Buffer &buffer) {
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

    // failed to open device or device stopped working
    loop.exit();
}

Coroutine echo(Loop &loop, Buffer &buffer) {
    while (buffer.ready()) {
        // receive something
        co_await buffer.read();

        // send it back
        co_await buffer.write();
    }

    // failed to open device or device stopped working
    loop.exit();
}

#ifdef NATIVE
// pass two serial port devices as arguments, e.g. "\\\\.\\COM9" "\\\\.\\COM10"
int main(int argc, char **argv) {
    if (argc < 3)
        return 1;
    drivers.init(argv[1], argv[2]);
#else
int main() {
#endif
    //debug::setRed();
    writeRead(drivers.loop, drivers.buffer1);
    echo(drivers.loop, drivers.buffer2);

    drivers.loop.run();

    return 0;
}
