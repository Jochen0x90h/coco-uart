#pragma once

#include <coco/String.hpp>
#include <coco/UartMonitor.hpp>
#include <coco/platform/Loop_Win32.hpp> // includes Windows.h
#include <map>


namespace coco {

/// @brief Implementation of UartMonitor using SetupAPI and registry.
/// Polls every second for new devices.
class UartMonitor_SetupAPI : public UartMonitor, public Loop_Win32::DeviceHandler {
public:

    UartMonitor_SetupAPI(Loop_Win32 &loop);

    ~UartMonitor_SetupAPI() override;

    void listenAdd(std::function<void (DevicePath, String)> function, Action action = Action::ENUMERATE_MONITOR) override;
    void listenRemove(std::function<void (DevicePath)> function) override;

protected:
    // Loop_Win32::DeviceHandler methods
    void onDeviceChange(Loop_Win32::DeviceType type, bool add, DevicePath path) override;


    Loop_Win32 &loop_;

    std::vector<std::function<void (DevicePath, String)>> addListeners_;
    std::vector<std::function<void (DevicePath)>> removeListeners_;
};

} // namespace coco
