//===========================================================================//
//
// Copyright (C) 2020 LP-Research Inc.
//
// This file is part of OpenZen, under the MIT License.
// See https://bitbucket.org/lpresearch/openzen/src/master/LICENSE for details
// SPDX-License-Identifier: MIT
//
//===========================================================================//
#include "components/ImuIg1Component.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <string>
#include <iostream>

#include "ZenTypesHelpers.h"
#include "SensorManager.h"
#include "properties/ImuSensorPropertiesV0.h"
#include "components/SensorParsingUtil.h"

namespace zen
{
    ImuIg1Component::ImuIg1Component(std::unique_ptr<ISensorProperties> properties, SyncedModbusCommunicator& communicator, unsigned int,
        bool hasFirstGyro, bool hasSecondGyro) noexcept
        : SensorComponent(std::move(properties))
        , m_communicator(communicator)
        , m_hasFirstGyro(hasFirstGyro)
        , m_hasSecondGyro(hasSecondGyro)
    {}

    ZenSensorInitError ImuIg1Component::init() noexcept
    {
        // Once setup is done, reset to streaming
        if (ZenError_None != m_properties->setBool(ZenImuProperty_StreamData, true))
            return ZenSensorInitError_RetrieveFailed;

        return ZenSensorInitError_None;
    }

    ZenError ImuIg1Component::processData(uint16_t function, gsl::span<const std::byte> data) noexcept
    {
        const auto property = static_cast<EDevicePropertyV1>(function);
        switch (property)
        {
        case EDevicePropertyV1::GetImuId:
        case EDevicePropertyV1::GetFilterMode:
        case EDevicePropertyV1::GetAccRange:
        case EDevicePropertyV1::GetGyrRange:
        case EDevicePropertyV1::GetMagRange:
        case EDevicePropertyV1::GetGyrThreshold:
        case EDevicePropertyV1::GetGyrFilter:
        case EDevicePropertyV1::GetEnableGyrAutoCalibration:
        case EDevicePropertyV1::GetImuTransmitData:
        case EDevicePropertyV1::GetStreamFreq:
        case EDevicePropertyV1::GetCanStartId:
        case EDevicePropertyV1::GetCanBaudRate:
        case EDevicePropertyV1::GetCanDataPrecision:
        case EDevicePropertyV1::GetCanMode:
        case EDevicePropertyV1::GetCanHeartbeat:
            if (data.size() != sizeof(uint32_t))
                return ZenError_Io_MsgCorrupt;
            return m_communicator.publishResult(function, ZenError_None, *reinterpret_cast<const uint32_t*>(data.data()));

        case EDevicePropertyV1::GetCanMapping:
            if (data.size() != sizeof(uint32_t) * 16)
                return ZenError_Io_MsgCorrupt;
            return m_communicator.publishArray(function, ZenError_None, gsl::make_span(reinterpret_cast<const int32_t*>(data.data()), 16));
        default:
            return ZenError_Io_UnsupportedFunction;
        }
    }

    nonstd::expected<ZenEventData, ZenError> ImuIg1Component::processEventData(ZenEventType eventType, gsl::span<const std::byte> data) noexcept
    {
        switch (eventType)
        {
        case ZenEventType_ImuData:
            return parseSensorData(data);
            break;

        default:
            return nonstd::make_unexpected(ZenError_UnsupportedEvent);
        }
    }

    nonstd::expected<ZenEventData, ZenError> ImuIg1Component::parseSensorData(gsl::span<const std::byte> data) const noexcept
    {
        // check SET_LPBUS_DATA_PRECISION how to get the 32/16 bit setting, needs to be cached !

        // Any properties that are retrieved here should be cached locally, because it
        // will take too much time to retrieve from the sensor!

        // Units will always be converted to degrees and degrees/s no matter how the
        // IG1 output is actually configured. OpenZen output unit is always degrees

        ZenEventData eventData;
        ZenImuData& imuData = eventData.imuData;
        imuDataReset(imuData);

        const auto begin = data.begin();
        const auto size = data.size();

        if ((long unsigned int)std::distance(begin, data.begin() + sizeof(uint32_t)) > size)
            return nonstd::make_unexpected(ZenError_Io_MsgCorrupt);

        const auto isRadOutput = *m_properties->getBool(ZenImuProperty_DegRadOutput);
        const auto isLowPrecisionOutput = *m_properties->getBool(ZenImuProperty_OutputLowPrecision);

        // The sensor data arrives in an order documented on https://lp-research.atlassian.net/wiki/spaces/LKB/pages/1255145474/LPMS-IG1+User+Manual#Sensor-Measurement-Data (2022/2/25)
        // Timestamp needs to be multiplied by 0.002 to convert to seconds
        // also output the raw framecount as provided by the sensor
        // This will always be 32-bit, independent if low precission mode is selected
        sensor_parsing_util::parseAndStoreScalar(data, &imuData.frameCount);
        imuData.timestamp = imuData.frameCount * 0.002;

        if (auto enabled = sensor_parsing_util::readVector3IfAvailable(ZenImuProperty_OutputRawAcc,
            m_properties, isLowPrecisionOutput, 1000.0f, data, &imuData.aRaw[0])) {}
        else {
            return nonstd::make_unexpected(enabled.error());
        }

        if (auto enabled = sensor_parsing_util::readVector3IfAvailable(ZenImuProperty_OutputAccCalibrated,
            m_properties, isLowPrecisionOutput, 1000.0f, data, &imuData.a[0])) {}
        else {
            return nonstd::make_unexpected(enabled.error());
        }

        // gyro raw value
        if (m_hasFirstGyro) {
            if (auto enabled = sensor_parsing_util::readVector3IfAvailable(ZenImuProperty_OutputRawGyr0,
                m_properties, isLowPrecisionOutput, isRadOutput ? 1000.0f: 10.0f, data, &imuData.g1Raw[0])) {
                sensor_parsing_util::radToDegreesIfNeededVector3(m_properties, &imuData.g1Raw[0]);
            } else {
                return nonstd::make_unexpected(enabled.error());
            }
        }

        if (m_hasSecondGyro) {
            if (auto enabled = sensor_parsing_util::readVector3IfAvailable(ZenImuProperty_OutputRawGyr1,
                m_properties, isLowPrecisionOutput, isRadOutput ? 100.0f: 10.0f, data, &imuData.g2Raw[0])) {
                sensor_parsing_util::radToDegreesIfNeededVector3(m_properties, &imuData.g2Raw[0]);
            } else {
                return nonstd::make_unexpected(enabled.error());
            }
        }

        // gyro bias calibrated value
        if (m_hasFirstGyro) {
            if (auto enabled = sensor_parsing_util::readVector3IfAvailable(ZenImuProperty_OutputGyr0BiasCalib,
                m_properties, isLowPrecisionOutput, isRadOutput ? 1000.0f: 10.0f, data, &imuData.g1BiasCalib[0])) {
                    sensor_parsing_util::radToDegreesIfNeededVector3(m_properties, &imuData.g1BiasCalib[0]);
                }
            else {
                return nonstd::make_unexpected(enabled.error());
            }
        }

        if (m_hasSecondGyro) {
            if (auto enabled = sensor_parsing_util::readVector3IfAvailable(ZenImuProperty_OutputGyr1BiasCalib,
                m_properties, isLowPrecisionOutput, isRadOutput ? 100.0f: 10.0f, data, &imuData.g2BiasCalib[0])) {
                    sensor_parsing_util::radToDegreesIfNeededVector3(m_properties, &imuData.g2BiasCalib[0]);
                }
            else {
                return nonstd::make_unexpected(enabled.error());
            }
        }

        // gyro aliment calibrated value
        // alignment calibration also contains the static calibration correction
        if (m_hasFirstGyro) {
            if (auto enabled = sensor_parsing_util::readVector3IfAvailable(ZenImuProperty_OutputGyr0AlignCalib,
                m_properties, isLowPrecisionOutput, isRadOutput ? 1000.0f: 10.0f, data, &imuData.g1[0])) {
                    sensor_parsing_util::radToDegreesIfNeededVector3(m_properties, &imuData.g1[0]);
                }
            else {
                return nonstd::make_unexpected(enabled.error());
            }
        }

        if (m_hasSecondGyro) {
            if (auto enabled = sensor_parsing_util::readVector3IfAvailable(ZenImuProperty_OutputGyr1AlignCalib,
                m_properties, isLowPrecisionOutput, isRadOutput ? 100.0f: 10.0f, data, &imuData.g2[0])) {
                    sensor_parsing_util::radToDegreesIfNeededVector3(m_properties, &imuData.g2[0]);
                }
            else {
                return nonstd::make_unexpected(enabled.error());
            }
        }

        if (auto enabled = sensor_parsing_util::readVector3IfAvailable(ZenImuProperty_OutputRawMag,
            m_properties, isLowPrecisionOutput, 100.0f, data, &imuData.bRaw[0])) {}
        else {
            return nonstd::make_unexpected(enabled.error());
        }

        if (auto enabled = sensor_parsing_util::readVector3IfAvailable(ZenImuProperty_OutputMagCalib,
            m_properties, isLowPrecisionOutput, 100.0f, data, &imuData.b[0])) {}
        else {
            return nonstd::make_unexpected(enabled.error());
        }

        // this is the angular velocity which takes into account when an orientation offset was
        // done
        if (auto enabled = sensor_parsing_util::readVector3IfAvailable(ZenImuProperty_OutputAngularVel,
            m_properties, isLowPrecisionOutput, 100.0f, data, &imuData.w[0])) {
                sensor_parsing_util::radToDegreesIfNeededVector3(m_properties, &imuData.w[0]);
            }
        else {
            return nonstd::make_unexpected(enabled.error());
        }

        if (auto enabled = sensor_parsing_util::readVector4IfAvailable(ZenImuProperty_OutputQuat,
            m_properties, isLowPrecisionOutput, 10000.0f, data, &imuData.q[0])) {}
        else {
            return nonstd::make_unexpected(enabled.error());
        }

        if (auto enabled = sensor_parsing_util::readVector3IfAvailable(ZenImuProperty_OutputEuler,
            m_properties, isLowPrecisionOutput, isRadOutput ? 10000.0f: 100.0f, data, &imuData.r[0])) {
                sensor_parsing_util::radToDegreesIfNeededVector3(m_properties, &imuData.r[0]);
            }
        else {
            return nonstd::make_unexpected(enabled.error());
        }

        if (auto enabled = sensor_parsing_util::readVector3IfAvailable(ZenImuProperty_OutputLinearAcc,
            m_properties, isLowPrecisionOutput, 1000.0f, data, &imuData.linAcc[0])) {}
        else {
            return nonstd::make_unexpected(enabled.error());
        }

        // At this time, Pressure and Altitude are not suppported by the IG1 firmware
        // and are not outputted. Still we will keep this code in place because the
        // output bits and data fields are still present
        if (auto enabled = sensor_parsing_util::readScalarIfAvailable(ZenImuProperty_OutputPressure,
            m_properties, data, &imuData.pressure, isLowPrecisionOutput)) {}
        else {
            return nonstd::make_unexpected(enabled.error());
        }

        if (auto enabled = sensor_parsing_util::readScalarIfAvailable(ZenImuProperty_OutputAltitude,
            m_properties, data, &imuData.altitude, isLowPrecisionOutput)) {}
        else {
            return nonstd::make_unexpected(enabled.error());
        }

        if (auto enabled = sensor_parsing_util::readScalarIfAvailable(ZenImuProperty_OutputTemperature,
            m_properties, data, &imuData.temperature, isLowPrecisionOutput, 100.0f)) {}
        else {
            return nonstd::make_unexpected(enabled.error());
        }

        return eventData;
    }
}
