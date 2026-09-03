#pragma once

#include <coco/DevicePath.hpp>
#include <coco/enum.hpp>
#include <filesystem>
#include <functional>


namespace coco {

/// @brief Monitor for UART devices.
/// When a device is added or removed, a callback gets invoked.
class UartMonitor {
public:
    enum class Action {
        ENUMERATE = 1,
        MONITOR = 2,
        ENUMERATE_MONITOR = 3
    };

    virtual ~UartMonitor() {};

    /// @brief Listen on add events
    /// @param action Action to perform (enumerate, monitor or both)
    /// @param function Callback function with path to device and name
    virtual void listenAdd(std::function<void (DevicePath, String)> function, Action action =
        Action::ENUMERATE_MONITOR) = 0;

    /// @brief Listen on remove events
    /// @param action Action to perform (enumerate, monitor or both)
    /// @param function Callback function with path to device
    virtual void listenRemove(std::function<void (DevicePath)> function) = 0;
};
COCO_ENUM(UartMonitor::Action);

} // namespace coco
