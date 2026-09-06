# IO Reference

## Communication

`BaseChannel` is the transport-independent contract for `Start()`, `Send()`, `Read()`, `IsConnected()`, and `Stop()`.

- `GenericTCP` adds TCP discovery hooks used by Motion Link.
- `GenericSerial` provides the serial transport abstraction.
- `ESP32TCP` implements the embedded TCP server and mDNS discovery.
- `ESP32Serial` adapts an Arduino `HardwareSerial` instance.

A channel is responsible for delivering bytes. It does not interpret Motion Bridge payloads. The Link protocol's complete-frame requirement must be satisfied by the channel or by a framing layer above it.

## Sensors

`ISensor` defines sensor metadata and lifecycle behavior. `BaseSensor<T>` adds typed readings and serialization. `GenericEncoder` defines the quadrature encoder contract, while `ESP32Encoder` maps it to ESP32 pulse counters.

Typical encoder setup:

```cpp
auto encoder = ESP32Encoder::Create("right encoder", {17, 16});
if (!encoder->Start())
{
    // Do not start odometry when encoder hardware is unavailable.
}
```

Only the platform-specific adapter knows the GPIO and peripheral details. Robot services should depend on `GenericEncoder` or another interface rather than an ESP32 class.

## Actuators

`IActuator` defines lifecycle and serialized command behavior. `BaseActuator<T>` adds typed commands. `GenericMotor` models a floating-point motor command, `GenericDCMotor` defines dual-pin DC motor behavior, and `ESP32DCMotor` implements GPIO/PWM output.

```cpp
auto motor = ESP32DCMotor::Create("left motor", {25, 26});
if (!motor->Start())
{
    // Keep the robot disabled until the actuator is ready.
}
motor->SetCommand(0.0f);
```

The application owns the motor safety policy. Stop motors when a sensor, controller, or communication failure invalidates a command.

## Logger

`Logger` is a framework service initialized with a communication channel. The `LOG_TRACE`, `LOG_INFO`, `LOG_WARN`, and `LOG_ERROR` macros provide the normal application-facing interface.

```cpp
BaseChannelHandle logger = ESP32Serial::Create(Serial, 921600);
LOG_START(logger, LogLevel::TRACE);
LOG_INFO("robot initialized");
```

Start the logger before relying on diagnostic output during hardware initialization.

## Wheel conversion

`Wheel` converts encoder measurements into wheel distance and speed using encoder resolution and wheel radius. The radius and track width must use the same linear units as odometry and navigation.

```cpp
auto rightWheel = Wheel::Create(rightEncoder, 350, 25.0f);
auto leftWheel = Wheel::Create(leftEncoder, 350, 25.0f);
rightWheel->Start();
leftWheel->Start();
```

Calibrate the conversion with measured travel rather than assuming nominal hardware dimensions.

## Platform boundary

Keep ESP32-specific classes at the application or platform composition boundary. Core robot algorithms should consume generic interfaces so they can be tested with simulated channels, sensors, and actuators.
