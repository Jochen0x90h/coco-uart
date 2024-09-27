#include <Rs485SendTest.hpp>
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
    while (buffer.ready()) {
        debug::toggleGreen();
        co_await buffer.writeString("Hello RS485");
        co_await loop.sleep(50ms);
    }

    // failed to open device or device stopped working
    loop.exit();
}


#ifdef NATIVE
// pass serial port device as argument, e.g. "\\\\.\\COM9"
int main(int argc, char **argv) {
    if (argc < 2)
        return 1;
    drivers.init(argv[1]);
#else
int main() {
#endif
    send(drivers.loop, drivers.rs485, drivers.sendBuffer);

    drivers.loop.run();

    return 0;
}
