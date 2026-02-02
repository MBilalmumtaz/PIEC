//===========================================================================//
//
// Copyright (C) 2020 LP-Research Inc.
//
// This file is part of OpenZen, under the MIT License.
// See https://bitbucket.org/lpresearch/openzen/src/master/LICENSE for details
// SPDX-License-Identifier: MIT
//
//===========================================================================//

// explicitly request the C++17 version of the interface
#define OPENZEN_CXX17
#include "OpenZen.h"

#include <iostream>

using namespace zen;

/**
 * This example demonstrates the C++17 interface of the OpenZen library.
 * 
 * Check our docs for more https://lpresearch.bitbucket.io/openzen/latest/getting_started.html
 */
int main(int argc, char* argv[])
{
    // enable resonable log output for OpenZen
    ZenSetLogLevel(ZenLogLevel_Info);

    // create OpenZen Clien
    auto [clientError, client] = make_client();

    if (clientError) {
        std::cout << "Cannot create OpenZen client" << std::endl;
        return clientError;
    }

    // connect to sensor on IO System by the sensor name
    // more on https://lpresearch.bitbucket.io/openzen/latest/io_systems.html
    auto [obtainError, sensor] = client.obtainSensorByName("SiUsb", "lpmscu2000573", 921600);
    // auto [obtainError, sensor] = client.obtainSensorByName("WindowsDevice", "\\\\.\\COM7", 921600);
    if (obtainError)
    {
        std::cout << "Cannot connect to sensor" << std::endl;
        client.close();
        return obtainError;
    }

    // check that the sensor has an IMU component
    auto imu = sensor.getAnyComponentOfType(g_zenSensorType_Imu);

    if (!imu)
    {
        std::cout << "Connected sensor has no IMU" << std::endl;
        client.close();
        return ZenError_WrongSensorType;
    }

    // set and get current streaming frequency
    auto error = imu->setInt32Property(ZenImuProperty_SamplingRate, 50);
    if (error) {
        std::cout << "Error setting streaming frequency" << std::endl;
        client.close();
        return error;
    }

    auto freqPair = imu->getInt32Property(ZenImuProperty_SamplingRate);
    if (freqPair.first) {
        std::cout << "Error fetching streaming frequency" << std::endl;
        client.close();
        return freqPair.first;
    }
    std::cout << "Streaming frequency: " << freqPair.second << std::endl;

    // toggle on/off of a particular data output (linAcc is not ON by default)
    error = imu->setBoolProperty(ZenImuProperty_OutputLinearAcc, true);
    if (error) {
        std::cout << "Error toggling ON linear acc data output" << std::endl;
        client.close();
        return error;
    }

    // readout up to 200 samples from the IMU
    // note that there are 2 gyro fields in the IMU data structure (ZenImuData struct in include/ZenTypes.h)
    // please refer to your sensor's manual for correct retrieval of gyro data
    for (int i = 0; i < 200; i++) {
        auto event = client.waitForNextEvent();
        if (event->component.handle == imu->component().handle) {
            std::cout << "> Acceleration: \t x = " << event->data.imuData.a[0]
                << "\t y = " << event->data.imuData.a[1]
                << "\t z = " << event->data.imuData.a[2] << std::endl;
            
            // depending on sensor, gyro data is outputted to g1, g2 or both
            // read more on https://lpresearch.bitbucket.io/openzen/latest/getting_started.html#id1
            std::cout << "> Gyro 1: \t\t x = " << event->data.imuData.g1[0]
                << "\t y = " << event->data.imuData.g1[1]
                << "\t z = " << event->data.imuData.g1[2] << std::endl;
        }
    }

    client.close();
    std::cout << "Sensor connection closed" << std::endl;
    return 0;
}
