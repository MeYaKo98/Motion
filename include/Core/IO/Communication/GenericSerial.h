/**
 * @file GenericSerial.h
 * @brief Header file for the GenericSerial class, providing a generic serial communication interface.
 * @details Extends BaseChannel to represent a standard serial (UART) communication endpoint.
 */

#pragma once

#include "Core/IO/Communication/BaseChannel.h"

namespace Motion::Core::IO {

/**
 * @class GenericSerial
 * @brief A generic implementation of a serial communication channel.
 * @details This class extends `BaseChannel` to provide a standardized abstraction for
 *          serial (UART) communication. It is a base class for platform-specific implementations
 *          (e.g., ESP32Serial, ARMSerial) that handle the actual hardware details.
 *
 *          **Serial Communication Context:**
 *          Serial communication (UART) is one of the most common communication protocols in embedded systems:
 *          - Simple: Only two data lines (TX, RX) plus ground
 *          - Reliable: Usually includes error checking (parity, checksums at higher layers)
 *          - Compatible: Widely supported across microcontrollers and PCs
 *          - Standard Baud Rates: 9600, 14400, 19200, 38400, 57600, 115200 bps, etc.
 *
 *          **Use Cases in Robotics:**
 *          - Communication with sensors (GPS, IMU, range sensors)
 *          - Debug output and logging
 *          - Command input from operator/PC
 *          - Data streaming for telemetry
 *
 * @note **Template vs. Non-Template:** This is a non-template base class because serial communication
 *       always deals with byte streams (uint8_t). Template specialization isn't needed.
 *
 * @note **Inheritance Chain:** GenericSerial → BaseChannel → abstract interface
 *
 * @see BaseChannel for the general communication channel interface
 * @see ESP32Serial for an ESP32-specific implementation example
 */
class GenericSerial : public BaseChannel {
public:
    /**
     * @brief Virtual destructor.
     * @details Ensures proper polymorphic destruction of derived class instances.
     *          The default implementation is sufficient for most cases unless a derived
     *          class allocates resources in the constructor.
     *
     * @note Default implementation `= default` delegates to compiler-generated destructor,
     *       which calls the parent class destructor. This is appropriate for simple classes
     *       without heap allocations.
     */
    virtual ~GenericSerial() = default;

protected:
    /**
     * @brief Protected default constructor.
     * @details Initializes a new GenericSerial instance with a specified baud rate.
     *          Protected because this is an abstract base class; direct instantiation is not intended.
     *          Derived concrete classes (ESP32Serial, etc.) call this constructor.
     *
     * @param baudRate The baud rate for serial communication (bits per second).
     *                 Common values: 9600, 19200, 38400, 57600, 115200, 230400, 460800.
     *                 The actual baud rate must match the receiver's configuration, or communication fails.
     *                 Must be a positive value > 0.
     *
     * @post `_baudRate` is initialized to the provided value.
     * @post `BaseChannel::_started` is initialized to false (by BaseChannel constructor).
     *
     * @note **Baud Rate Selection:**
     *       - 9600: Very reliable, low speed (useful for long cables)
     *       - 115200: Very common for USB and modern boards
     *       - 230400-460800: High speed for short distances/high data rates
     *       Choose based on your cable length and data volume requirements.
     *
     * @note **Typical Usage in Derived Classes:**
     *       ```cpp
     *       class ESP32Serial : public GenericSerial {
     *           ESP32Serial(uint32_t baudRate) : GenericSerial(baudRate) { }
     *       };
     *       ```
     *
     * @warning **Baud Rate Mismatch:** If the configured baud rate doesn't match the receiving device's
     *          baud rate, communication will fail with garbled data. Always verify baud rate configuration.
     *
     * @warning **Velocity Limits:** Very high baud rates (> 921600) may be unreliable on some platforms
     *          due to clock divider limitations or electromagnetic interference. Test before production use.
     */
    GenericSerial(uint32_t baudRate) : _baudRate(baudRate), BaseChannel() {};

    /**
     * @brief The configured baud rate for serial communication.
     * @details Stored for reference and configuration of the underlying hardware.
     *          Derived classes read this value and configure their hardware accordingly.
     * @note This is a read-only member once set in the constructor.
     * @note Units: bits per second (bps). Common range: 1200 to 3000000 bps.
     */
    uint32_t _baudRate;
};

/**
 * @brief Smart handle (shared_ptr) for GenericSerial instances.
 * @details Manages the lifetime of serial channel objects, enabling automatic cleanup
 *          when the last reference is destroyed. Recommended for use throughout the application.
 *
 * @note Using shared_ptr allows multiple subsystems to reference the same serial port safely.
 */
using GenericSerialHandle = ChannelPointer(GenericSerial);

} // namespace Motion::Core::IO