#pragma once

#ifdef _WIN32
#include "UartMonitor_WinUSB.hpp"
namespace coco {
using UartMonitor_native = UartMonitor_WinUSB;
}
#endif
#ifdef __linux__
#include "UartMonitor_udev.hpp"
namespace coco {
using UartMonitor_native = UartMonitor_udev;
}
#endif
