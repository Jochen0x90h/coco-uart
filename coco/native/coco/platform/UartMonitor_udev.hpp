#pragma once

#include <coco/String.hpp>
#include <coco/UartMonitor.hpp>
#include <coco/platform/NativeFile.hpp>
#include <coco/platform/Loop_io_uring.hpp>
#include <libudev.h>


namespace coco {

/// @brief Implementation of UartMonitor using udev.
///
class UartMonitor_udev : public UartMonitor, public Loop_io_uring::CompletionHandler {
public:

    UartMonitor_udev(Loop_io_uring &loop);

    ~UartMonitor_udev() override;

    void listenAdd(std::function<void (const std::filesystem::path &, String)> function, Action action = Action::ENUMERATE_MONITOR) override;
    void listenRemove(std::function<void (const std::filesystem::path &)>);

protected:
    void onCompletion(io_uring_cqe &cqe, int index) override;

    Loop_io_uring &loop_;
    struct udev *udev_;
    struct udev_monitor* mon_;
    std::vector<std::function<void (const std::filesystem::path &, String)>> addListeners_;
    std::vector<std::function<void (const std::filesystem::path &)>> removeListeners_;
};

} // namespace coco
