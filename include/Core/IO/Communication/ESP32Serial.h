/**
 * @file .h
 * @brief Header file for the ESP32Serial communication channel.
 */

#pragma once

#include <Arduino.h>
#include "Core/IO/Communication/GenericSerial.h"

namespace Motion::Core::IO {

/**
 * @brief A communication channel implementation using ESP32 HardwareSerial.
 * @details This class wraps the Arduino `HardwareSerial` object (e.g., `Serial`, `Serial1`, `Serial2`)
 *          to provide a standardized interface. It allows the Motion Framework
 *          to communicate over UART using the standard Arduino API.
 *
 * @note This class relies on the Arduino framework's `HardwareSerial` implementation.
 * @warning **Thread Safety:** The underlying Arduino `HardwareSerial` is generally **not** thread-safe.
 *          If this channel is accessed from multiple FreeRTOS tasks, the caller must ensure proper
 *          mutual exclusion (e.g., using a mutex) before calling `Send` or `Read`.
 */
class ESP32Serial : public GenericSerial {
public:
    /**
     * @details Initializes the wrapper with a specific serial port and baud rate.
     *          The serial port is not opened until `Start()` is called.
     *
     * @param serial Reference to the global `HardwareSerial` object (e.g., `Serial` for USB, `Serial1` for UART pins).
     *               Defaults to `Serial`.
     * @param baudRate The baud rate for communication (e.g., 9600, 115200). Defaults to 115200.
     */
    ESP32Serial(HardwareSerial& serial = Serial, uint32_t baudRate = 115200);

    /**
     * @brief Destroys the ESP32Serial object.
     * @details Ensures the channel is stopped and resources are released.
     */
    virtual ~ESP32Serial() override;

    /**
     * @brief Starts the serial communication.
     * @details Calls `begin()` on the underlying `HardwareSerial` object with the configured baud rate.
     *          Sets the internal state to connected.
     * @return true if the serial port was successfully initialized; false otherwise.
     */
    bool Start() override;

    /**
     * @brief Sends data over the serial port.
     * @details Writes the specified buffer to the UART transmission buffer.
     *          This operation may block if the hardware TX buffer is full.
     *
     * @param data Pointer to the buffer containing the data to send.
     * @param length The number of bytes to send.
     * @return size_t The actual number of bytes written to the serial port.
     */
    size_t Send(const uint8_t* data, size_t length) override;

    /**
     * @brief Reads data from the serial port.
     * @details Checks for available bytes in the UART RX buffer and reads up to `bufferSize` bytes.
     *          This method is non-blocking; it returns immediately with the number of bytes currently available.
     *
     * @param buffer Pointer to the destination buffer where read data will be stored.
     * @param bufferSize The maximum number of bytes to read (size of the buffer).
     * @return size_t The actual number of bytes read. Returns 0 if no data is available.
     */
    size_t Read(uint8_t* buffer, size_t bufferSize) override;

    /**
     * @brief Checks if the serial connection is active.
     * @details For `HardwareSerial`, this typically checks if the port is initialized and valid.
     * @return true if the serial port is open and ready; false otherwise.
     */
    bool IsConnected() override;

    /**
     * @brief Stops the serial communication.
     */
    void Stop() override;

protected:
    /**
     * @brief Reference to the underlying Arduino HardwareSerial object.
     */
    HardwareSerial& _serial;
};

} // namespace Motion::Core::IO