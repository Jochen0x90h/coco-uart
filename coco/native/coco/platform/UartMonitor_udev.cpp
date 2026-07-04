#include "UartMonitor_udev.hpp"
#include <coco/platform/NativeFile.hpp>
#include <coco/debug.hpp>


namespace coco {

namespace {

    const char *subsystem = "tty";
    const char *devtype = nullptr;

    bool filter(struct udev_device *dev) {
        struct udev_device *parent = udev_device_get_parent(dev);
        if (!parent)
            return false;
        const char *driver = udev_device_get_driver(parent);
        if (!driver)
            return false;
        if (String(driver) == "port") {
            auto iomem = udev_device_get_sysattr_value(dev, "iomem_base");
            auto irq = udev_device_get_sysattr_value(dev, "irq");
            if (iomem == nullptr || irq == nullptr)
                return false;
            if (String(iomem) == "0x0" || String(irq) == "0")
                return false;
        }
        return true;
    }

} // namespace

UartMonitor_udev::UartMonitor_udev(Loop_io_uring &loop)
    : loop_(loop)
{
    udev_ = udev_new();
    mon_ = udev_monitor_new_from_netlink(udev_, "udev");

    udev_monitor_filter_add_match_subsystem_devtype(mon_, subsystem, devtype);
    udev_monitor_enable_receiving(mon_);

    // poll for events
    int fd = udev_monitor_get_fd(mon_);
    //fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK);
    loop.poll(*this, fd, POLLIN);
}

UartMonitor_udev::~UartMonitor_udev() {
    udev_unref(udev_);
}

void UartMonitor_udev::listenAdd(std::function<void (const std::filesystem::path &, String)> function, Action action) {
    if ((action & Action::ENUMERATE) != 0) {
        auto udev = udev_;
        auto enumerate = udev_enumerate_new(udev);
        udev_enumerate_add_match_subsystem(enumerate, subsystem);
        udev_enumerate_scan_devices(enumerate);
        struct udev_list_entry* devices = udev_enumerate_get_list_entry(enumerate);
        struct udev_list_entry* entry;
        udev_list_entry_foreach(entry, devices) {
            const char* path = udev_list_entry_get_name(entry);
            struct udev_device* dev = udev_device_new_from_syspath(udev, path);
            if (dev) {
                const char* devnode = udev_device_get_devnode(dev);
                struct udev_device *parent = udev_device_get_parent(dev);
                if (devnode && filter(dev)) {
                    // call user function
                    std::filesystem::path path = devnode;
                    function(path, path.filename().string());
                }
                udev_device_unref(dev);
            }
        }
        udev_enumerate_unref(enumerate);
    }

    if ((action & Action::MONITOR) != 0) {
        addListeners_.push_back(function);
    }
}

void UartMonitor_udev::listenRemove(std::function<void (const std::filesystem::path &)> function) {
    removeListeners_.push_back(function);
}


void UartMonitor_udev::onCompletion(io_uring_cqe &cqe, int index) {
    if (cqe.res & POLLIN) {
        // poll again
        int fd = udev_monitor_get_fd(mon_);
        loop_.poll(*this, fd, POLLIN);

        struct udev_device* dev = udev_monitor_receive_device(mon_);
        if (dev) {
            const char *action = udev_device_get_action(dev);
            const char* devnode = udev_device_get_devnode(dev);
            if (action && devnode && filter(dev)) {
                if (String(action) == "add") {
                    // call add listeners
                    for (auto &function : addListeners_) {
                        std::filesystem::path path = devnode;
                        function(path, path.filename().string());
                    }
                } else if (String(action) == "remove") {
                    // call remove listeners
                    for (auto &function : removeListeners_) {
                        function(devnode);
                    }
                }
            }

            udev_device_unref(dev);
        }
    }
}

} // namespace coco
