.. _examples-label:

###########################
Examples for OpenZen Usage
###########################

Basic examples
===============

For connecting to a single sensor, toggling sensor settings and output sensor data, please refer to :ref:`getting-started-label`.
If you prefer to study and run a full code example, please have a look
at this `example source file <https://bitbucket.org/lpresearch/openzen/src/master/examples/main.cpp>`_.

Connecting multiple Sensors
===========================

Its possible to connect multiple sensors with one OpenZen instance and event loops. Simply connect
to multiple sensors and store the sensor's handle:

.. code-block:: cpp

    auto sensorPairA = client.obtainSensorByName("SiUsb", "lpmscu2000574", 921600);
    auto& sensorA = sensorPairA.second;

    auto sensorPairB = client.obtainSensorByName("SiUsb", "lpmscu2000573", 921600);
    auto& sensorB = sensorPairB.second;

In your event loop, now check which sensor the last received event is orginating from:

.. code-block:: cpp

    auto event = client.waitForNextEvent();

    if (sensorA.sensor() == event.second.sensor) {
        std::cout << "Data from Sensor A" << std::endl;
    } else if (sensorB.sensor() == event.second.sensor) {
        std::cout << "Data from Sensor B" << std::endl;
    }

Synchronizing multiple Sensors
==============================

If multiple sensor are connected to the same OpenZen instance, they can be synchronized by putting
them into synchronization mode and then sending the command to leave synchronization mode at the same
time. The result of this operation will be that the ``timestamp`` and ``frameCount`` values returned by each
sensor will be in the same time frame. However, this method is a software synchronization and does not
account for the delay of the transport layer (USB, Bluetooth etc.) so the accuracy of this synchronization
is limited by this fact. In our experience, the software synchronization can achieve a synchronization better
than 5 milliseconds.

**C++ example code:**

.. code-block:: cpp

    // set both sensors in synchronization mode
    imu_component1.executeProperty(ZenImuProperty_StartSensorSync);
    imu_component2.executeProperty(ZenImuProperty_StartSensorSync);

    // wait a moment for the synchronization commands to arrive
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // set both sensors back to normal mode
    imu_component1.executeProperty(ZenImuProperty_StopSensorSync);
    imu_component2.executeProperty(ZenImuProperty_StopSensorSync);

    // start receiving regular events from the sensors

**Python example code:**

.. code-block:: python

    # set both sensors in synchronization mode
    imu_component1.execute_property(openzen.ZenImuProperty.StartSensorSync)
    imu_component2.execute_property(openzen.ZenImuProperty.StartSensorSync)

    # wait a moment for the synchronization commands to arrive
    time.sleep(3)

    # set both sensors back to normal mode
    imu_component1.execute_property(openzen.ZenImuProperty.StopSensorSync)
    imu_component2.execute_property(openzen.ZenImuProperty.StopSensorSync)

    # start receiving regular events from the sensors

Manual Gyroscope Calibration
============================

LPMS sensors include an advanced automatic calibration for the gyroscope bias
during the operation of the sensor. This calibration model will detect if
the sensor is at rest and automatically recalibrate the gyroscope bias without
any user intervention. This mode is suited well for most application areas.

However, in some application domains controlling the gyroscope bias calibration
manually can provide better results. Two examples are:

- The sensor is in constant motion so the automatic calibration will never be
  able to start.
- Slow and steady moving platforms might make the automatic calibration start
  even if the platform is actually not at rest but slowly moving and degrading
  the result of the automatic calibration.

Therefore, we provide an option to manually start the gyroscope bias calibration.
In this mode, the user has to ensure that the sensor is at rest for 6 seconds after
the calibration has been triggered. Furthermore, the automatic bias calibration needs
to be disabled so the manual calibration is not accidentally overwritten.

**C++ example code:**

.. code-block:: cpp

    // disable automatic calibration
    imu.setBoolProperty(ZenImuProperty_GyrUseAutoCalibration, false);
    std::cout << "Starting gyroscope calibration, don't move sensor" << std::endl;
    // start manual calibration
    imu.executeProperty(ZenImuProperty_CalibrateGyro);
    std::this_thread::sleep_for(std::chrono::seconds(6));
    std::cout << "Gyroscope calibration completed" << std::endl;

**Python example code:**

.. code-block:: python

    # disable automatic calibration
    imu.set_bool_property(openzen.ZenImuProperty.GyrUseAutoCalibration, False)
    print("Starting gyroscope calibration, don't move sensor")
    # start manual calibration
    imu.execute_property(openzen.ZenImuProperty.CalibrateGyro)
    time.sleep(6)
    print("Gyroscope calibration completed")
