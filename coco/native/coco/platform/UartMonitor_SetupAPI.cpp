#include <coco/platform/WindowsDef.hpp>
#include <windows.h>
#include <setupapi.h>
#include <initguid.h>
#include <ntddser.h> // GUID_DEVINTERFACE_COMPORT
#include <usbiodef.h>
#include <coco/platform/WindowsUndef.hpp>

#include "UartMonitor_SetupAPI.hpp"
#include <coco/debug.hpp>


namespace coco {

namespace {

    // buffer for device path and string descriptor
    union Buffer {
        SP_DEVICE_INTERFACE_DETAIL_DATA_W devicePath;
        uint16_t space[2 + 128];

        Buffer() {};
    };

} // namespace

UartMonitor_SetupAPI::UartMonitor_SetupAPI(Loop_Win32 &loop)
    : loop_(loop)
{
    onTimeout();
}

UartMonitor_SetupAPI::~UartMonitor_SetupAPI() {
}

void UartMonitor_SetupAPI::listenAdd(std::function<void (const std::filesystem::path &, String)> function, Action action) {
    if ((action & Action::ENUMERATE) != 0) {
        for (auto &p : deviceInfos_) {
            auto &deviceInfo = p.second;
            if (!deviceInfo.name.empty()) {
                // call user function
                function(p.first, deviceInfo.name);
            }
        }
    }
    if ((action & Action::MONITOR) != 0) {
        addListeners_.push_back(function);
    }
}

void UartMonitor_SetupAPI::listenRemove(std::function<void (const std::filesystem::path &)> function) {
    removeListeners_.push_back(function);
}

void UartMonitor_SetupAPI::onTimeout() {
    // restart timeout
    loop_.invoke(*this, 1s);

    // flag all devices
    for (auto &p : deviceInfos_) {
        p.second.flag = true;
    }

    // enumerate devices
    HDEVINFO devs = SetupDiGetClassDevsW(nullptr, nullptr, nullptr, DIGCF_ALLCLASSES | DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
    int index = 0;
    SP_DEVINFO_DATA deviceData;
    deviceData.cbSize = sizeof(SP_DEVINFO_DATA);
    while (SetupDiEnumDeviceInfo(devs, index, &deviceData)) {
        ++index;

        // get interface data
        SP_DEVICE_INTERFACE_DATA interfaceData;
        interfaceData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);
        if (!SetupDiEnumDeviceInterfaces(devs, &deviceData, &GUID_DEVINTERFACE_COMPORT, 0, &interfaceData)) {
            continue;
        }

        // buffer for device path and string descriptor
        Buffer buffer;

        // get device path
        buffer.devicePath.cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_W);
        SP_DEVINFO_DATA devInfoData;
        devInfoData.cbSize = sizeof(SP_DEVINFO_DATA);
        if (!SetupDiGetDeviceInterfaceDetailW(devs, &interfaceData, &buffer.devicePath, sizeof(buffer), nullptr, &devInfoData)) {
            // error
            continue;
        }

        // check if device is new
        wchar_t *path = buffer.devicePath.DevicePath;
        auto [it, inserted] = deviceInfos_.emplace(path, DeviceInfo{});
        auto &deviceInfo = it->second;
        //debug::out << it->first.string() << '\n';

        if (inserted) {
            // found a new device, path has the form \\?\usb#vid_1915&pid_1337#5&41045ef&0&4#{a5dcbf10-6530-11d2-901f-00c04fb951ed}

            // get registry key
            HKEY regKey = SetupDiOpenDevRegKey(devs, &devInfoData, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
            if (regKey == INVALID_HANDLE_VALUE) {
                continue;
            }

            // get port name
            wchar_t portName[64];
            DWORD valueSize = sizeof(portName);
            DWORD valueType = 0;
            LSTATUS status = RegQueryValueExW(regKey, L"PortName", NULL, &valueType, (LPBYTE)portName, &valueSize);
            RegCloseKey(regKey);
            if (status != ERROR_SUCCESS || valueType != REG_SZ) {
                continue;
            }
            int utf8Length = WideCharToMultiByte(CP_UTF8, 0, (wchar_t *)portName, wcslen(portName), nullptr, 0,
                nullptr, nullptr);
            if (utf8Length <= 0)
                continue;
            deviceInfo.name.assign(utf8Length, '\0');
            WideCharToMultiByte(CP_UTF8, 0, (wchar_t *)portName, wcslen(portName), deviceInfo.name.data(), utf8Length,
                nullptr, nullptr);

            // call add listeners
            for (auto &function : addListeners_) {
                function(it->first, deviceInfo.name);
            }
        }
        deviceInfo.flag = false;
    }

    // detect removed devices
    auto it = deviceInfos_.begin();
    while (it != deviceInfos_.end()) {
        auto current = it;
        ++it;
        if (current->second.flag) {
            auto &deviceInfo = current->second;
            if (!deviceInfo.name.empty()) {
                // call remove listeners
                for (auto &function : removeListeners_) {
                    function(current->first);
                }
            }

            // erase
            deviceInfos_.erase(current);
        }
    }
}

} // namespace coco
