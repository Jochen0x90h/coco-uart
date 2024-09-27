#pragma once

#ifdef _WIN32
#include "Uart_Win32.hpp"
namespace coco {
using Uart_native = Uart_Win32;
}
#endif
