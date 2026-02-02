#define NOMINMAX
#include "windows.h"

#include "EnumerateSerialPorts.h"

// This is derived from CEnumerateSerial found here: http://www.naughter.com/enumser.html
// referenced here https://stackoverflow.com/a/1394301/8680401.
//
// Copyright header on the original source:
/*
Copyright(c) 1998 - 2019 by PJ Naughter(Web: www.naughter.com, Email : pjna@naughter.com)

All rights reserved.

Copyright / Usage Details :

You are allowed to include the source code in any product (commercial, shareware, freeware or otherwise) 
when your product is released in binary form. You are allowed to modify the source code in any way you want 
except you cannot modify the copyright details at the top of each module. If you want to distribute source 
code with your application, then you are only allowed to distribute versions released by the author. This is 
to maintain a single distribution point for the source code. 
*/
// I don't think this qualifies as a version of the original code in the sense described above, as it
// cannot be used in the original's stead, this code isn't a drop-in replacement even for
// CEnumerateSerial::UsingRegistry().
// It has at least the following behavioral differences:
//  1) no inconsistent trailing NUL-characters in output
//  2) output is sorted
//  3) restricted to one-byte characters
//  4) doesn't call SetLastError()
// Also there are a number of implementation differences:
//  1) no _in_, _out_ etc annotations, no warning suppressions
//  2) no ATL used
//  3) automatic registry handle cleanup via gsl::finally
//  4) registry type check already during length check
//  5) main loop structure simplified
//  6) indentation, function names, "::" prefixes, etc. in line with OpenZen habits

#include <algorithm>
#include <optional>
#include <string>
#include <vector>

#include "gsl/gsl"

namespace {
    // Gets the string valued registry key sans trailing NULs.
    std::optional<std::string> GetRegistryString(HKEY key, LPCSTR lpValueName)
    {

        // Query for the size of the registry value
        ULONG nBytes = 0;
        DWORD dwType = 0;
        if (::RegQueryValueExA(key, lpValueName,
                nullptr, &dwType, nullptr, &nBytes) != ERROR_SUCCESS)
            return std::nullopt;

        if (dwType != REG_SZ && dwType != REG_EXPAND_SZ)
            return std::nullopt;

        // Allocate enough bytes for the return value
        std::string result(static_cast<size_t>(nBytes), '\0');

        // Call again, now actually loading the string.
        if (::RegQueryValueExA(key, lpValueName, nullptr, &dwType,
                reinterpret_cast<LPBYTE>(result.data()), &nBytes) != ERROR_SUCCESS)
            return std::nullopt;

        // COM1 at least has a NUL in the end, clean up.
        result.erase(std::find(result.begin(), result.end(), '\0'), result.end());
        return result;
    }

    auto enumerateAllVcpPorts()
        -> std::vector<zen::PortAndSerial>
    {    
        std::vector<zen::PortAndSerial> vPortAndSerialAll;
        HKEY hKey = 0;
        auto closeReg = gsl::finally([&hKey]() { if (hKey) RegCloseKey(hKey); });
        // This key contains all devices with this known vendor and product id. pid = EA60 is VCP mode, EA61 is USBXpress
        if (::RegOpenKeyExA(HKEY_LOCAL_MACHINE, R"(SYSTEM\CurrentControlSet\Enum\USB\VID_10C4&PID_EA60)", 0,
                KEY_QUERY_VALUE | KEY_READ, &hKey) != ERROR_SUCCESS)
            return {};

        // Get the max value name and max value lengths
        DWORD cSubKeys = 0;
        if (::RegQueryInfoKeyA(hKey, nullptr, nullptr, nullptr, &cSubKeys, nullptr,
                nullptr, nullptr, nullptr, nullptr, nullptr, nullptr) != ERROR_SUCCESS)
            return {};

        for (DWORD i = 0; i < cSubKeys; i++) {
            std::vector<char> vSerialNumber(16383);
            DWORD len = (DWORD)vSerialNumber.size();
            if (RegEnumKeyExA(hKey, i,
                    vSerialNumber.data(), &len,
                    nullptr, nullptr, nullptr, nullptr) != ERROR_SUCCESS)
                continue;
            auto serialNumber = std::string(vSerialNumber.data(), len);

            std::string sKey = serialNumber + "\\Device Parameters";
            if (HKEY hSubKey = 0
                ; ::RegOpenKeyExA(hKey, sKey.c_str(), 0, KEY_QUERY_VALUE | KEY_READ, &hSubKey) == ERROR_SUCCESS) {
                if (auto pPortName = GetRegistryString(hSubKey, "PortName"))
                    vPortAndSerialAll.push_back({ std::move(pPortName.value()), std::move(serialNumber) });
                RegCloseKey(hSubKey);
            }
        }
        return vPortAndSerialAll;
    }

    auto enumerateAllActiveComPorts()
        -> std::optional<std::vector<std::string>>
    {
        HKEY hKey = 0;
        auto closeReg = gsl::finally([&hKey]() { if (hKey) RegCloseKey(hKey); });
        if (::RegOpenKeyExA(HKEY_LOCAL_MACHINE, R"(HARDWARE\DEVICEMAP\SERIALCOMM)", 0,
                KEY_QUERY_VALUE, &hKey) != ERROR_SUCCESS)
            return std::nullopt;

        // Get the max value name and max value lengths
        DWORD dwMaxValueNameLen = 0;
        if (::RegQueryInfoKeyA(hKey, nullptr, nullptr, nullptr, nullptr, nullptr,
               nullptr, nullptr, &dwMaxValueNameLen, nullptr, nullptr, nullptr) != ERROR_SUCCESS)
            return std::nullopt;

        std::vector<std::string> result;
        const DWORD dwMaxValueNameSizeInChars = dwMaxValueNameLen + 1; //Include space for the null terminator

        // Enumerate all the values underneath HKEY_LOCAL_MACHINE\HARDWARE\DEVICEMAP\SERIALCOMM
        DWORD dwIndex = 0;
        std::vector<char> valueName(dwMaxValueNameSizeInChars, 0);
        while (true) {
            DWORD dwValueNameSize = dwMaxValueNameSizeInChars;
            if (::RegEnumValueA(hKey, dwIndex++, valueName.data(), &dwValueNameSize,
                    nullptr, nullptr, nullptr, nullptr) != ERROR_SUCCESS)
                break;

            std::string sPortName;
            if (auto pPortName = ::GetRegistryString(hKey, valueName.data())) {
                result.push_back(std::move(pPortName.value()));
            }
        }
        return result;
    }
}

bool zen::EnumerateSerialPorts(std::vector<PortAndSerial>& vAvailablePortAndSerial)
{
    // Find all known SiLabs devices in VCP mode.
    std::vector<PortAndSerial> vPortAndSerialAll = enumerateAllVcpPorts();

    // Find all live COM port.
    auto pActivePorts = enumerateAllActiveComPorts();
    if (!pActivePorts)
        return false;

    // VCP ports which are live are the sensors we can connect to.
    vAvailablePortAndSerial = {};    
    for (auto& pas : vPortAndSerialAll) {
        if (auto it = std::find(pActivePorts->begin(), pActivePorts->end(), pas.port)
            ; it != pActivePorts->end())
            vAvailablePortAndSerial.emplace_back(pas);
    }

    // Sort the output.
    std::sort(vAvailablePortAndSerial.begin(), vAvailablePortAndSerial.end(),
        [](const PortAndSerial& left, const PortAndSerial& right) -> bool {
            // Strings are either COMx, COMxx or COMxxx with no leading zeros.
            // COMx comes before COMxx, COMxxx etc.
            if (left.port.size() != right.port.size())
                return left.port.size() < right.port.size();
            // Compare the digit strings which are guaranteed to be of the same length.
            return left.port.substr(3) < right.port.substr(3);
        });

    return true;
}
