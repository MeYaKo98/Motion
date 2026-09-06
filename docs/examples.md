# Examples and Templates

The examples directory contains small composition templates rather than board-specific applications. Replace the placeholder channel and robot types with implementations from the target project.

## Template selection

| Template | Use it when |
|---|---|
| [`MotionLinkTcpTemplate.cpp`](MotionLinkTcpTemplate.cpp) | The robot accepts remote clients over TCP and mDNS discovery. |
| [`MotionLinkSerialTemplate.cpp`](MotionLinkSerialTemplate.cpp) | The robot is controlled through a UART or other serial channel. |
| [`MotionLinkCustomCommandTemplate.cpp`](MotionLinkCustomCommandTemplate.cpp) | The application exposes a command outside the framework services. |
| [`CompleteDifferentialDriveMotionLink.cpp`](CompleteDifferentialDriveMotionLink.cpp) | The application combines ESP32 hardware, odometry, controllers, navigation, Wi-Fi, and TCP Link. |

## Template conventions

- Initialize hardware before exposing its services.
- Check transport startup before calling `Spin()`.
- Keep fast callbacks short and non-blocking.
- Validate every request payload length before reading it.
- Use application command values at or above `UserCommandStart`.
- Keep objects captured by callbacks alive for the lifetime of Link.
- Use the same declared units for dimensions, odometry, profiles, and navigation.

The templates intentionally use placeholder type names so they can be copied into a board project without assuming a particular hardware implementation. The complete differential-drive template contains ESP32-specific composition and is the closest match to a working board application.

## Complete differential-drive template

[`CompleteDifferentialDriveMotionLink.cpp`](CompleteDifferentialDriveMotionLink.cpp) follows the full application composition used by the reference ESP32 robot.

1. Creates the logger, motors, encoders, wheels, and odometry.
2. Creates one PID controller per wheel.
3. Creates a trapezoidal profile and combines settle and oscillation stop conditions.
4. Creates the differential-drive navigation service.
5. Starts hardware and connects to Wi-Fi.
6. Starts the TCP Motion Link server and attaches navigation callbacks.
7. Enters the blocking Link receive loop.

The file is a board template. Replace GPIO assignments, calibration values, and the Wi-Fi placeholders before deploying. Never commit real network credentials.
