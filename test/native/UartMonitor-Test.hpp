#pragma once

#include <coco/platform/Loop_native.hpp>
#include <coco/platform/UartMonitor_native.hpp>


using namespace coco;

/// @brief Drivers for UartMonitor-Test
///
struct Drivers {
    Loop_native loop;

    UartMonitor_native monitor{loop};
};

Drivers drivers;
