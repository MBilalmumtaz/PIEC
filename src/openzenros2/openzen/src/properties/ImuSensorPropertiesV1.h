//===========================================================================//
//
// Copyright (C) 2020 LP-Research Inc.
//
// This file is part of OpenZen, under the MIT License.
// See https://bitbucket.org/lpresearch/openzen/src/master/LICENSE for details
// SPDX-License-Identifier: MIT
//
//===========================================================================//

#ifndef ZEN_PROPERTIES_IMUSENSORPROPERTIESV1_H_
#define ZEN_PROPERTIES_IMUSENSORPROPERTIESV1_H_

#include <array>
#include <cstring>
#include <utility>

#include "InternalTypes.h"
#include "ZenTypes.h"

namespace zen
{
    namespace imu::v1
    {
        constexpr EDevicePropertyV1 mapCommand(ZenProperty_t command)
        {
            switch (command)
            {
            /* case ZenImuProperty_PollSensorData:
                return EDevicePropertyV1::GetRawSensorData; */

            case ZenImuProperty_CalibrateGyro:
                return EDevicePropertyV1::StartGyroCalibration;

            case ZenImuProperty_ResetOrientationOffset:
                return EDevicePropertyV1::ResetOrientationOffset;

            default:
                return EDevicePropertyV1::Ack;
            }
        }

        constexpr EDevicePropertyV1 map(ZenProperty_t property, bool isGetter)
        {
            const auto set_or = [isGetter](EDevicePropertyV1 prop) {
                return isGetter ? EDevicePropertyV1::Ack : prop;
            };
            const auto get_set = [isGetter] (EDevicePropertyV1 x, EDevicePropertyV1 y) {
                return isGetter ? x : y;
            };

            switch (property)
            {
            case ZenImuProperty_Id:
                return get_set(EDevicePropertyV1::GetImuId, EDevicePropertyV1::SetImuId);

            case ZenImuProperty_SamplingRate:
                return get_set(EDevicePropertyV1::GetStreamFreq, EDevicePropertyV1::SetStreamFreq);

            case ZenImuProperty_FilterMode:
                return get_set(EDevicePropertyV1::GetFilterMode, EDevicePropertyV1::SetFilterMode);

            case ZenImuProperty_OrientationOffsetMode:
                return set_or(EDevicePropertyV1::SetOrientationOffsetMode);

            case ZenImuProperty_AccRange:
                return get_set(EDevicePropertyV1::GetAccRange, EDevicePropertyV1::SetAccRange);

            case ZenImuProperty_GyrRange:
                return get_set(EDevicePropertyV1::GetGyrRange, EDevicePropertyV1::SetGyrRange);

            case ZenImuProperty_GyrUseAutoCalibration:
                return get_set(EDevicePropertyV1::GetEnableGyrAutoCalibration, EDevicePropertyV1::SetEnableGyrAutoCalibration);

            case ZenImuProperty_GyrUseThreshold:
                return get_set(EDevicePropertyV1::GetGyrThreshold, EDevicePropertyV1::SetGyrThreshold);

            case ZenImuProperty_GyrFilter:
                return get_set(EDevicePropertyV1::GetGyrFilter, EDevicePropertyV1::SetGyrFilter);

            case ZenImuProperty_MagRange:
                return get_set(EDevicePropertyV1::GetMagRange, EDevicePropertyV1::SetMagRange);

            case ZenImuProperty_DegRadOutput:
                return get_set(EDevicePropertyV1::GetDegGradOutput, EDevicePropertyV1::SetDegGradOutput);

            // in Ig1, this is not part of the output flags but its own command
            case ZenImuProperty_OutputLowPrecision:
                return get_set(EDevicePropertyV1::GetLpBusDataPrecision, EDevicePropertyV1::SetLpBusDataPrecision);

            /* CAN bus properties */
            case ZenImuProperty_CanStartId:
                return get_set(EDevicePropertyV1::GetCanStartId, EDevicePropertyV1::SetCanStartId);

            case ZenImuProperty_CanBaudrate:
                return get_set(EDevicePropertyV1::GetCanBaudRate, EDevicePropertyV1::SetCanBaudRate);

            case ZenImuProperty_CanPointMode:
                return get_set(EDevicePropertyV1::GetCanDataPrecision, EDevicePropertyV1::SetCanDataPrecision);

            case ZenImuProperty_CanChannelMode:
                return get_set(EDevicePropertyV1::GetCanMode, EDevicePropertyV1::SetCanMode);

            case ZenImuProperty_CanMapping:
                return get_set(EDevicePropertyV1::GetCanMapping, EDevicePropertyV1::SetCanMapping);

            case ZenImuProperty_CanHeartbeat:
                return get_set(EDevicePropertyV1::GetCanHeartbeat, EDevicePropertyV1::SetCanHeartbeat);

            /* UART output properties */
            case ZenImuProperty_UartBaudRate:
                return get_set(EDevicePropertyV1::GetUartBaudrate, EDevicePropertyV1::SetUartBaudrate);

            case ZenImuProperty_UartFormat:
                return get_set(EDevicePropertyV1::GetUartBaudrate, EDevicePropertyV1::SetUartFormat);

            default:
                return EDevicePropertyV1::Ack;
            }
        }

        // this list is directly from the IG1 documentation 20220106
        // https://lp-research.atlassian.net/wiki/spaces/LKB/pages/1255145474/LPMS-IG1+User+Manual
        constexpr std::array<int32_t, 6> supportedSamplingRates{5, 10, 50, 100, 250, 500};
        constexpr std::array<int32_t, 5> supportedFilterModes{0, 1, 2, 3, 4};
        constexpr std::array<int32_t, 4> supportedAccRanges{2, 4, 8, 16};
        constexpr std::array<int32_t, 3> supportedGyrRanges{400, 1000, 2000};
        constexpr std::array<int32_t, 2> supportedMagRanges{2, 8};

        inline gsl::span<const int32_t> getPropertyOptions(ZenProperty_t property)
        {
            switch (property)
            {
            case ZenImuProperty_SupportedSamplingRates:
            case ZenImuProperty_SamplingRate:
                return gsl::make_span(supportedSamplingRates.data(), supportedSamplingRates.size());
                
            case ZenImuProperty_SupportedFilterModes:
            case ZenImuProperty_FilterMode:
                return gsl::make_span(supportedFilterModes.data(), supportedFilterModes.size());
                
            case ZenImuProperty_AccSupportedRanges:
            case ZenImuProperty_AccRange:
                return gsl::make_span(supportedAccRanges.data(), supportedAccRanges.size());
                
            case ZenImuProperty_GyrSupportedRanges:
            case ZenImuProperty_GyrRange:
                return gsl::make_span(supportedGyrRanges.data(), supportedGyrRanges.size());
                
            case ZenImuProperty_MagSupportedRanges:
            case ZenImuProperty_MagRange:
                return gsl::make_span(supportedMagRanges.data(), supportedMagRanges.size());
            
            default:
                throw std::runtime_error("Invalid property name for getPropertyOptions()");
            }
        }

        inline std::pair<ZenError, size_t> getSupportedOptions(ZenProperty_t property, gsl::span<int32_t> buffer)
        {
            auto supported = getPropertyOptions(property);

            if (buffer.size() < supported.size())
                return std::make_pair(ZenError_BufferTooSmall, supported.size());

            if (buffer.data() == nullptr)
                return std::make_pair(ZenError_IsNull, supported.size());

            std::copy(supported.begin(), supported.end(), buffer.begin());
            return std::make_pair(ZenError_None, supported.size());
        }

        inline int32_t mapToSupportedOption(ZenProperty_t property, int32_t value)
        {
            switch (property) {
                case ZenImuProperty_SamplingRate:
                case ZenImuProperty_FilterMode:
                case ZenImuProperty_AccRange:
                case ZenImuProperty_GyrRange:
                case ZenImuProperty_MagRange:
                {
                    auto supported = getPropertyOptions(property);
                    for (const auto &option : supported)
                    {
                        if (value > option)
                            continue;
                        return option;
                    }
                    return supported[supported.size() - 1];
                }
                
                default:
                    return value;
            }
        }
    }
}

#endif
