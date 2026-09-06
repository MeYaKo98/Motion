# Motion Robotics Framework

<p align="center">
  <img src="docs/images/logo.svg" width="500" alt="Motion Robotics Logo">
</p>

<p align="center">
    <img src="https://img.shields.io/badge/PlatformIO-Framework-orange.svg" alt="PlatformIO">
    <img src="https://img.shields.io/badge/Language-C%2B%2B17-blue.svg" alt="C++17">
</p>

## Overview

A lightweight, high-performance robotics framework designed for embedded applications and microcontrollers. Motion provides hardware interfaces, robot services, and communication middleware without coupling application logic to a single board or transport.

## Documentation

The documentation is organized around the system architecture:

- [Documentation landing page](docs/mainpage.md): Start here for the component map and startup order.
- [Getting started](docs/getting-started.md): Build prerequisites and first robot composition.
- [Architecture guide](docs/architecture.md): Layer responsibilities, task boundaries, and extension points.
- [Core concepts](docs/core-concepts.md): Handles, lifecycle, concurrency, units, and ownership.
- [IO reference](docs/io-reference.md): Communication, sensors, actuators, logging, and wheels.
- [Motion pipeline](docs/motion-pipeline.md): Odometry, controllers, profiles, stop conditions, and navigation.
- [Motion Link protocol](docs/protocol.md): Wire frames, commands, payloads, and status values.
- [Examples guide](docs/examples.md): TCP, serial, and custom command templates.
- [Generated API documentation](docs/html/index.html): Doxygen output after generation.

## Key Features

- **Interface-driven design:** Decouples hardware implementations from robot logic through IO base classes.
- **Low memory footprint:** Suitable for ESP32, STM32, Arduino, and similar embedded targets.
- **Robot service layer:** Provides odometry, navigation, drive, position, and stop-condition abstractions.
- **Motion Link middleware:** Exposes robot services over TCP or serial using a small request/response protocol.
- **PlatformIO ready:** Supports the PlatformIO ecosystem and unit testing.

## Project Structure

- `include/` - Public interfaces and core definitions.
- `src/` - Framework implementations and service adapters.
- `examples/` - Documentation-oriented code templates.
- `docs/` - Doxygen configuration, framework guides, assets, and generated output.

<p align="center">
  <i>Developed for precision motion and autonomous robot control.</i>
</p>