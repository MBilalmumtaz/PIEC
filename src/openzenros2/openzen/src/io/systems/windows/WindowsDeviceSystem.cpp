//===========================================================================//
//
// Copyright (C) 2020 LP-Research Inc.
//
// This file is part of OpenZen, under the MIT License.
// See https://bitbucket.org/lpresearch/openzen/src/master/LICENSE for details
// SPDX-License-Identifier: MIT
//
//===========================================================================//

#include "io/systems/windows/WindowsDeviceSystem.h"
#include "io/interfaces/windows/WindowsDeviceInterface.h"
#include "EnumerateSerialPorts.h"

#include <spdlog/spdlog.h>

namespace zen
{
    ZenError WindowsDeviceSystem::listDevices(std::vector<ZenSensorDesc>& outDevices)
    {
        std::vector<PortAndSerial> serialPorts;
        if (!EnumerateSerialPorts(serialPorts))
            return ZenError_Device_ListingFailed;

        for (auto it = serialPorts.begin(); it != serialPorts.end(); ++it) {
            ZenSensorDesc desc{}; // Zeros all char buffers -> no need to copy NUL below.

            const std::string name("COM PORT #" + it->port.substr(3));
            assert(name.length() <= sizeof desc.name - 1);
            std::memcpy(desc.name, name.c_str(), name.length());

            std::memcpy(desc.serialNumber, it->serialNumber.c_str(),
               (std::min)(sizeof desc.serialNumber - 1, it->serialNumber.length()));
            static_assert(sizeof desc.ioType >= sizeof WindowsDeviceSystem::KEY);
            std::memcpy(desc.ioType, WindowsDeviceSystem::KEY, sizeof(WindowsDeviceSystem::KEY));

            const std::string filename("\\\\.\\" + it->port);
            assert(filename.length() <= sizeof desc.identifier - 1);
            std::memcpy(desc.identifier, filename.c_str(), filename.length());

            desc.baudRate = 921600;
            outDevices.emplace_back(desc);
        }
        return ZenError_None;
    }

    nonstd::expected<std::unique_ptr<IIoInterface>, ZenSensorInitError> WindowsDeviceSystem::obtain(const ZenSensorDesc& desc, IIoDataSubscriber& subscriber) noexcept
    {
        auto handle = ::CreateFileA(desc.identifier, GENERIC_READ | GENERIC_WRITE,
            0, nullptr, OPEN_EXISTING, 0, nullptr);
        if (handle == INVALID_HANDLE_VALUE) {
            spdlog::error("Cannot open Windows IO file {0}", desc.identifier);
            return nonstd::make_unexpected(ZenSensorInitError_InvalidAddress);
        }

        // configure the COM Port. This needs to be done before CreateEvent is called. Otherwise the
        // settings are not directly used
        DCB config;
        if (!::GetCommState(handle, &config)) {
            spdlog::error("Cannot get Windows IO comm state");
            return nonstd::make_unexpected(ZenSensorInitError_IoFailed);
        }

        config.fOutxCtsFlow = false;
        config.fOutxDsrFlow = false;
        config.fDtrControl = DTR_CONTROL_ENABLE;
        config.fOutX = false;
        config.fInX = false;
        config.fRtsControl = RTS_CONTROL_ENABLE;
        config.ByteSize = 8;
        config.Parity = NOPARITY;
        config.StopBits = ONESTOPBIT;

        if (!::SetCommState(handle, &config)) {
            spdlog::error("Cannot set Windows IO comm state");
            return nonstd::make_unexpected(ZenSensorInitError_IoFailed);
        }

        COMMTIMEOUTS timeoutConfig;
        timeoutConfig.ReadIntervalTimeout = 0;
        timeoutConfig.ReadTotalTimeoutMultiplier = 0;
        timeoutConfig.ReadTotalTimeoutConstant = 1;
        timeoutConfig.WriteTotalTimeoutMultiplier = 1;
        timeoutConfig.WriteTotalTimeoutConstant = 1;

        if (!::SetCommTimeouts(handle, &timeoutConfig)) {
            spdlog::error("Cannot set Windows IO comm timeouts");
            return nonstd::make_unexpected(ZenSensorInitError_IoFailed);
        }

        auto ioInterface = std::make_unique<WindowsDeviceInterface>(subscriber, desc.identifier, handle);

        // Note: Setting of baudrate is not needed here because its done by the connection negotiator
        spdlog::debug("Windows device interface for {0} created", desc.identifier);

        return ioInterface;
    }
}
