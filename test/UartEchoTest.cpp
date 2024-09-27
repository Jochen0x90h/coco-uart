#include <UartSendTest.hpp>
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


Coroutine echo(Loop &loop, Buffer &buffer) {
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
    echo(drivers.loop, drivers.sendBuffer);

    drivers.loop.run();
    return 0;
}
