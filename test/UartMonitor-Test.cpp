#include <UartMonitor-Test.hpp>
#include <coco/convert.hpp>
#include <coco/debug.hpp>


// Test UART monitor.
// Add and remove a USB to UART adapter and check the debug output for the device name and path.


using namespace coco;


int main() {
    debug::out << "UartMonitor-Test\n";

    drivers.monitor.listenAdd([](DevicePath device, String name) {
        debug::out << name << " (" << device << ")\n";
    });
    drivers.monitor.listenRemove([](DevicePath device) {
        debug::out << " (" << device << ")\n";
    });

    drivers.loop.run();

    return 0;
}
