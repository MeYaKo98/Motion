# Getting Started

## Prerequisites

Motion targets embedded applications built with PlatformIO and Arduino-compatible FreeRTOS environments. The public headers use nested namespace definitions, so configure the project for C++17 or a compiler mode that supports them.

For an ESP32 application, the usual prerequisites are:

- PlatformIO with the Espressif32 platform.
- An Arduino-compatible ESP32 board.
- A Motion library dependency available to the application.
- FreeRTOS task, queue, and synchronization primitives supplied by the board framework.
- Hardware-specific wiring and calibration values.

## Include the framework

Use the umbrella headers when an application needs the complete framework:

```cpp
#include <Motion/Core.h>
#include <Motion/Link.h>
```

Use narrower headers in library code when reducing compile-time dependencies is important:

```cpp
#include <Motion/Core/IO/Actuator/ESP32DCMotor.h>
#include <Motion/Core/IO/Sensor/ESP32Encoder.h>
#include <Motion/Core/Robot/Navigation/DifferentialDriveNavigation.h>
```

## Create a robot pipeline

A differential-drive robot typically composes the system in this order:

1. Create the logger, motors, and sensors.
2. Create wheels from encoder handles and calibrated dimensions.
3. Create and start odometry.
4. Create controllers, a motion profile, and a stop condition.
5. Create navigation from those dependencies.
6. Start a transport and attach services if remote control is required.

The complete composition is available in [CompleteDifferentialDriveMotionLink.cpp](CompleteDifferentialDriveMotionLink.cpp).

```cpp
GenericDCMotorHandle rightMotor =
    ESP32DCMotor::Create("right motor", {33, 32});
GenericDCMotorHandle leftMotor =
    ESP32DCMotor::Create("left motor", {25, 26});

GenericEncoderHandle rightEncoder =
    ESP32Encoder::Create("right encoder", {17, 16});
GenericEncoderHandle leftEncoder =
    ESP32Encoder::Create("left encoder", {27, 14});

WheelHandle rightWheel = Wheel::Create(rightEncoder, 350, 25.0f);
WheelHandle leftWheel = Wheel::Create(leftEncoder, 350, 25.0f);

auto odometry = DifferentialDriveOdometry::Create(120.0f, rightWheel, leftWheel);
odometry->Start();
```

The example above uses one consistent unit system. If wheel dimensions are millimeters, distances and profile values must also be millimeters; if they are meters, every related value must be meters.

## Add Motion Link

`Motion::Link` owns one active transport and blocks in `Spin()` after startup:

```cpp
if (!Motion::Link::StartTCP<ESP32TCP>())
{
    // Keep the robot in a safe state and report the startup failure.
    return;
}

Motion::Link::AttachCallbacks<DifferentialDriveNavigation>(navigation);
Motion::Link::Spin();
```

`DifferentialDriveNavigation` does not expose a `Start()` method. Start the hardware and odometry dependencies explicitly before attaching the navigation callbacks.

## Before deploying

- Replace all GPIO pins with the actual board wiring.
- Verify encoder phase order and wheel direction.
- Calibrate wheel radii, track width, and encoder resolution.
- Confirm all motion values use the same units.
- Replace Wi-Fi placeholders without committing credentials.
- Start motors, wheels, and odometry before issuing navigation commands.
- Add `ExternalStopCondition` callbacks if the client must use the `Stop` command.
