#pragma once

#if defined(_WIN32)
#include "Uart_Win32.hpp"
namespace coco {
using Uart_native = Uart_Win32;
}
#elif defined(__linux__)
#include "Uart_io_uring.hpp"
namespace coco {
using Uart_native = Uart_io_uring;
}
#endif
