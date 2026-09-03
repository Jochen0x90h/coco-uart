#include <coco/platform/WindowsDef.hpp>
#include <windows.h>
#include <setupapi.h>
#include <initguid.h> // DEFINE_GUID needed for GUIDs
#include <ntddser.h> // GUID_DEVINTERFACE_COMPORT
#include <devpkey.h> // DEVPKEY_Device_FriendlyName
#include <cfgmgr32.h>
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

    // discard high byte of wchar_t in-place and return as coco::String
    String toString(wchar_t *buffer) {
        char *begin = (char *)buffer;
        char *out = begin;
        while (*buffer != 0) {
            *out = *buffer;
            ++buffer;
            ++out;
        }
        *out = 0;
        return String(begin, out - begin);
    }
} // namespace

UartMonitor_SetupAPI::UartMonitor_SetupAPI(Loop_Win32 &loop)
    : loop_(loop)
{
    //onTimeout();
    loop.addDeviceHandler(*this);
}

UartMonitor_SetupAPI::~UartMonitor_SetupAPI() {
}

void UartMonitor_SetupAPI::listenAdd(std::function<void (DevicePath, String)> function, Action action) {
    if ((action & Action::ENUMERATE) != 0) {
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
            if (!SetupDiGetDeviceInterfaceDetailW(devs, &interfaceData, &buffer.devicePath, sizeof(buffer), nullptr, &devInfoData))
                continue;
            DevicePath path = buffer.devicePath.DevicePath;

            // get registry key
            HKEY regKey = SetupDiOpenDevRegKey(devs, &devInfoData, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
            if (regKey == INVALID_HANDLE_VALUE)
                continue;

            // get port name
            wchar_t portName[64];
            ULONG portNameSize = sizeof(portName);
            DWORD valueType = 0;
            LSTATUS status = RegQueryValueExW(
                regKey,
                L"PortName",
                NULL,
                &valueType,
                (LPBYTE)portName,
                &portNameSize);
            RegCloseKey(regKey);
            if (status != ERROR_SUCCESS || valueType != REG_SZ)
                continue;

            String name = toString(portName);

            // call user function
            function(path, name);
        }
    }
    if ((action & Action::MONITOR) != 0) {
        addListeners_.push_back(function);
    }
}

void UartMonitor_SetupAPI::listenRemove(std::function<void (DevicePath)> function) {
    removeListeners_.push_back(function);
}

void UartMonitor_SetupAPI::onDeviceChange(Loop_Win32::DeviceType type, bool add, DevicePath path) {
    if (type != Loop_Win32::DeviceType::COM)
        return;
    if (add) {
        // get device instance id
        wchar_t instanceId[MAX_DEVICE_ID_LEN] = {};
        ULONG instanceIdSize = sizeof(instanceId);
        DEVPROPTYPE propType;
        CONFIGRET ret = CM_Get_Device_Interface_PropertyW(
            path.c_str(),
            &DEVPKEY_Device_InstanceId,
            &propType,
            (PBYTE)instanceId,
            &instanceIdSize,
            0
        );
        if (ret != CR_SUCCESS)
            return;

        // get device node
        DEVNODE devNode;
        ret = CM_Locate_DevNodeW(&devNode, instanceId, CM_LOCATE_DEVNODE_NORMAL);
        if (ret != CR_SUCCESS) {
            wprintf(L"Fehler beim Lokalisieren des DevNodes. CR_Code: 0x%X\n", ret);
            return;
        }

        // open device parameters registry key of the device
        HKEY hKey = NULL;
        ret = CM_Open_DevNode_Key(
            devNode,
            KEY_READ,
            0,
            RegDisposition_OpenExisting,
            &hKey,
            CM_REGISTRY_HARDWARE // Greift auf die Hardware-Parameter des konkreten Interfaces zu
        );
        if (ret != CR_SUCCESS || hKey == NULL)
            return;

        // get port name (e.g. COM10) from registry
        wchar_t portName[64] = {};
        DWORD portNameSize = sizeof(portName);
        DWORD valueType = 0;
        LSTATUS regRet = RegQueryValueExW(
            hKey,
            L"PortName",
            NULL,
            &valueType,
            (LPBYTE)portName,
            &portNameSize
        );
        if (regRet != ERROR_SUCCESS || valueType != REG_SZ)
            return;

        String name = toString(portName);

        // call add listeners
        for (auto &function : addListeners_) {
            function(path, name);
        }
    } else {
        // call remove listeners
        for (auto &function : removeListeners_) {
            function(path);
        }
    }
}

} // namespace coco
