#pragma once

#include <coco/String.hpp>
#include <coco/UartMonitor.hpp>
#include <coco/platform/Loop_Win32.hpp> // includes Windows.h
#include <map>


namespace coco {

/// @brief Implementation of UartMonitor using SetupAPI and registry.
/// Polls every second for new devices.
class UartMonitor_SetupAPI : public UartMonitor, public Loop_Win32::TimeoutHandler {
public:

    UartMonitor_SetupAPI(Loop_Win32 &loop);

    ~UartMonitor_SetupAPI() override;

    void listenAdd(std::function<void (const std::filesystem::path &, String)> function, Action action = Action::ENUMERATE_MONITOR) override;
    void listenRemove(std::function<void (const std::filesystem::path &)>);

protected:
    void onTimeout() override;

    Loop_Win32 &loop_;

    struct DeviceInfo {
        // name (e.g. COM10)
        std::string name;

        // flag for "garbage collection" of devices
        bool flag;
    };
    std::map<std::filesystem::path, DeviceInfo> deviceInfos_;

    std::vector<std::function<void (const std::filesystem::path &, String)>> addListeners_;
    std::vector<std::function<void (const std::filesystem::path &)>> removeListeners_;
};

} // namespace coco
