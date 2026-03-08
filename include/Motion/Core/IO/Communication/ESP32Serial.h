/**
 * @file ESP32Serial.h
 * @brief ESP32 HardwareSerial communication channel implementation.
 * @details Provides a wrapper around Arduino's HardwareSerial for UART communication on ESP32.
 */

#pragma once

#include <Arduino.h>
#include "Motion/Core/IO/Communication/GenericSerial.h"

namespace Motion::Core::IO {

/**
 * @brief A communication channel implementation using ESP32 HardwareSerial.
 * @details This class wraps the Arduino `HardwareSerial` object (e.g., `Serial`, `Serial1`, `Serial2`, `Serial3`)
 *          to provide a standardized interface compatible with the BaseChannel abstraction.
 *          It allows the Motion Framework to communicate over UART using the standard Arduino API,
 *          enabling easy integration with existing Arduino sketches and libraries.
 *
 *          **ESP32 UART Hardware:**
 *          - 3 independent UART controllers (UART0, UART1, UART2)
 *          - Each with independent baud rate, data format, and flow control settings
 *          - UART0 typically used for monitor output (pins TX0=1, RX0=3)
 *          - UART1 and UART2 available for custom applications
 *
 *          **Arduino Serial Objects:**
 *          - `Serial`: Mapped to UART0 (USB communication via CH340 chip)
 *          - `Serial1`: Mapped to UART1 (pins GPIO17/GPIO16 by default, configurable)
 *          - `Serial2`: Mapped to UART2 (pins GPIO16/GPIO17 by default, configurable)
 *          - `Serial3`: Alternative name for UART2
 *
 *          **Common Usage in Robotics:**
 *          - Serial (UART0): Debug output and console communication
 *          - Serial1: GPS or GNSS receiver
 *          - Serial2: Lidar or range sensor
 *          - Multiple serial ports allow parallel communication with different devices
 *
 * @note **Arduino API:** This wrapper uses the Arduino `HardwareSerial` API, which is
 *       a high-level abstraction over the ESP-IDF UART drivers. The Arduino layer
 *       provides ease of use at some cost to lower-level control.
 *
 * @warning **Thread Safety:** The underlying Arduino `HardwareSerial` is **generally NOT thread-safe**.
 *          If this channel is accessed from multiple FreeRTOS tasks, the caller **must ensure
 *          proper mutual exclusion** (e.g., using a mutex) before calling `Send()` or `Read()`.
 *          Without synchronization, concurrent access causes:
 *          - Corrupted data in Rx/Tx buffers
 *          - Garbled output
 *          - Potential crashes
 *
 * @note **Baud Rate Limits:** Common baud rates are 9600, 19200, 38400, 57600, 115200, 230400, 460800.
 *       Very high baud rates (> 921600) may be unreliable on noisy connections or long cables.
 *
 * @note **Buffering:** HardwareSerial provides internal RX and TX buffers (typically 256 bytes).
 *       High-speed data or long messages may overflow buffers. Monitor buffer usage in applications
 *       with sustained high throughput.
 *
 * @see GenericSerial for the platform-independent serial interface
 * @see BaseChannel for the general communication channel interface
 */
class ESP32Serial;
    
/**
 * @brief Smart handle for ESP32Serial instances.
 * @details Uses smart pointers for automatic lifetime management and safe reference counting.
 */
using ESP32SerialHandle = ChannelPointer(ESP32Serial);

class ESP32Serial : public GenericSerial {
public:
    /**
     * @brief Factory method to create an ESP32Serial communication channel.
     * @details Creates a new ESP32Serial wrapper around a HardwareSerial object.
     *          Validates inputs and returns a smart pointer for safe lifetime management.
     *
     * @param serial Reference to the global `HardwareSerial` object to wrap.
     *               Common values:
     *               - `Serial`: UART0 (USB via CH340, default)
     *               - `Serial1`: UART1 (GPIO17/GPIO16 by default)
     *               - `Serial2`: UART2 (GPIO16/GPIO17 by default)
     *               Defaults to `Serial` if not specified.
     *
     * @param baudRate The baud rate for communication (bits per second).
     *                 Common values: 9600, 19200, 38400, 57600, 115200, 230400, 460800.
     *                 Must match the receiving device's baud rate.
     *                 Defaults to 115200 (very common for modern boards).
     *
     * @return ESP32SerialHandle A smart pointer to the newly created ESP32Serial instance.
     *
     * @post The instance is created but **NOT started**. Call Start() to initialize hardware.
     *
     * @note **Lazy Initialization:** The HardwareSerial object is not configured until Start() is called.
     *       This allows setting up the channel before the hardware is ready.
     *
     * @note **Usage Pattern:**
     *       ```cpp
     *       auto serial = ESP32Serial::Create(Serial1, 115200);
     *       if (serial->Start()) {
     *           uint8_t msg[] = "Hello";
     *           serial->Send(msg, 5);
     *       }
     *       serial->Stop();
     *       ```
     *
     * @note **Default Arguments:** If called without arguments, uses Serial at 115200 baud.
     *       Ideal for rapid prototyping:
     *       ```cpp
     *       auto serial = ESP32Serial::Create();  // Serial, 115200 baud
     *       ```
     *
     * @warning The provided `serial` reference must remain valid. Arduino's global `Serial`, `Serial1`, etc.
     *          are statically allocated and always valid, but custom HardwareSerial objects must be
     *          managed carefully.
     *
     * @warning Passing an invalid baud rate (e.g., 0) may cause unexpected behavior.
     *          Validate baud rates against your target device's specifications.
     */    
    static ESP32SerialHandle Create(HardwareSerial& serial = Serial, uint32_t baudRate = 115200);

    /**
     * @brief Destroys the ESP32Serial object.
     * @details Ensures the channel is stopped and resources are released.
     *          Calls Stop() to close the serial port if still open.
     *
     * @note The destructor automatically calls Stop(), so explicit Stop() before
     *       destruction is optional but recommended for clarity.
     */
    virtual ~ESP32Serial() override;

    /**
     * @brief Starts the serial communication channel.
     * @details Calls `begin()` on the underlying `HardwareSerial` object with the configured baud rate.
     *          This initializes the UART hardware, configures pin assignments, and opens the port.
     *          Sets the channel's internal state to "connected".
     *
     *          Initialization steps:
     *          1. `_serial.begin(_baudRate)` configures UART with the baud rate
     *          2. Internal `_started` flag set to true
     *          3. Port is ready for Send/Read operations
     *
     * @return true if the serial port was successfully initialized and is ready for I/O.
     * @return false if initialization failed (rare; usually indicates hardware issues).
     *
     * @post On success:
     *       - The underlying HardwareSerial is initialized with the configured baud rate
     *       - `_started` = true
     *       - `Send()` and `Read()` are operational
     *       - `IsConnected()` will return true
     *
     * @note **Idempotency:** Calling Start() twice typically has no harmful effect;
     *       the second call reconfigures the port and returns success.
     *
     * @note **USB to Serial:** If using `Serial` (UART0 with USB), the port may take
     *       1-3 seconds to connect depending on the operating system. On some boards,
     *       the CH340 USB-to-serial chip requires drivers.
     *
     * @note **Power Supply:** Ensure the device connected to the ESP32 UART is properly powered
     *       and wired before or immediately after calling Start(). Floating RX lines may cause noise.
     *
     * @note **Pin Configuration:** The Arduino layer automatically assigns pins for each UART:
     *       - Serial (UART0): TX=1, RX=3 (fixed, typically USB via CH340)
     *       - Serial1 (UART1): TX=17, RX=16 (default, can be reconfigured via setPins)
     *       - Serial2 (UART2): TX=16, RX=17 (default, can be reconfigured via setPins)
     *       For non-standard pin assignments, use the lower-level ESP-IDF API.
     *
     * @attention **Arduino HardwareSerial.begin() doesn't return a status;** it always returns
     *            void. This implementation returns true (success) because begin() doesn't fail
     *            in typical operation. Rare hardware failures are silent.
     *
     * @warning **No Error Reporting:** The Arduino HardwareSerial API doesn't report
     *          initialization failures. This method always returns true, even if the
     *          underlying initialization failed (rare edge case). If communication doesn't work,
     *          check baud rate and pin configuration.
     *
     * @warning **GPIO Conflicts:** If the assigned UART pins are in use by other peripherals
     *          (e.g., another UART, SPI, I2C), behavior is undefined. Check your pin assignments.
     *
     * @see Stop()
     * @see IsConnected()
     */
    bool Start() override;

    /**
     * @brief Sends data over the serial port.
     * @details Writes the specified buffer to the UART transmission buffer via the Arduino
     *          `write()` method. This operation may block if the hardware TX buffer is full,
     *          though Arduino typically uses interrupt-driven TX with buffering.
     *
     *          **Buffer Handling:**
     *          - Data is copied to the HardwareSerial TX buffer
     *          - TX ISR gradually transmits bytes at the configured baud rate
     *          - For a 115200 baud connection: ~12 bytes/millisecond transmission time
     *          - High-speed bursts may overflow the TX buffer if Send() isn't called frequently
     *
     * @param data Pointer to the buffer containing the data to send.
     *             Must not be nullptr. Must point to valid, readable memory.
     *
     * @param length The number of bytes to send from the buffer.
     *               Must be > 0 and consistent with the actual buffer size.
     *               Sending length > actual_buffer_size causes buffer overrun (undefined behavior).
     *
     * @return size_t The actual number of bytes written to the serial port buffer.
     *                - **Success:** Returns `length` if all bytes were accepted by the buffer
     *                - **Partial:** Returns < `length` if the TX buffer became full
     *                - **Failure:** Returns 0 if send failed (e.g., port not started)
     *
     *                For typical use, ensure the return value equals `length`, indicating all
     *                data was buffered successfully.
     *
     * @pre Start() must have been called successfully before this method.
     *      Calling Send() before Start() has undefined behavior (likely returns 0).
     *
     * @post The data bytes are queued in the HardwareSerial TX buffer.
     *       They will be transmitted by the UART hardware at the configured baud rate.
     *       Transmission may take microseconds to milliseconds depending on length and baud rate.
     *
     * @note **Non-Blocking Transmission:** This method queues data; actual transmission happens
     *       asynchronously via ISR. The method returns immediately without waiting for transmission.
     *
     * @note **Buffer Size:** HardwareSerial TX buffer is typically 256 bytes. Large messages
     *       (> 256 bytes) may overflow if Send() is called rapidly without spacing.
     *       Monitor return value to detect buffer overflows.
     *
     * @note **Baud Rate Impact:** Transmission time = length_bytes * 10_bits / baud_rate.
     *       At 115200 baud, a 256-byte message takes ~22 ms to transmit.
     *       High baud rates (460800) reduce this to ~5.5 ms.
     *
     * @note **Format:** Serial transmission uses standard UART framing (8 data bits, 1 stop bit,
     *       no parity by default). Higher layers must handle message framing/delimiters.
     *
     * @note **Thread-safety:** NOT thread-safe without external synchronization.
     *       Concurrent calls from multiple FreeRTOS tasks cause race conditions.
     *       Use a mutex to protect Send() calls if needed.
     *
     * @warning **Null Pointer:** Passing `data == nullptr` causes undefined behavior (crash).
     *          Always validate pointers.
     *
     * @warning **Buffer Overrun:** Passing `length` > actual_buffer_size causes a buffer overrun
     *          and undefined behavior (memory corruption, crash).
     *          Always allocate buffers correctly: `new uint8_t[length]`.
     *
     * @warning **Port Not Started:** If Start() fails or stop() was called, Send() fails silently
     *          (returns 0 or returns < length). Check IsConnected() before Send().
     *
     * @warning **Partial Sends:** If Send() returns < length, the TX buffer was full.
     *          Caller must retry the remaining bytes later. Implement retry logic for reliability.
     *
     * @see Read() for receiving data
     * @see IsConnected() to verify port is active
     */
    size_t Send(const uint8_t* data, size_t length) override;

    /**
     * @brief Reads data from the serial port.
     * @details Checks for available bytes in the UART RX buffer and reads up to `bufferSize` bytes.
     *          This method is **non-blocking**; it returns immediately with the number of bytes
     *          currently available (0 to `bufferSize`). If no data is available, it returns 0
     *          without blocking or waiting.
     *
     *          **Buffer Handling:**
     *          - RX ISR continuously copies bytes from the UART hardware FIFO to the HardwareSerial RX buffer
     *          - Read() drains bytes from the RX buffer up to `bufferSize`
     *          - If data arrives faster than Read() is called, the RX buffer may overflow
     *          - Overflow silently drops old data (no error reporting)
     *
     * @param buffer Pointer to the destination buffer where read data will be stored.
     *               Must not be nullptr. Must be large enough to hold `bufferSize` bytes.
     *               Buffer is allocated and managed by the caller.
     *
     * @param bufferSize The maximum number of bytes to read (capacity of the buffer).
     *                   Actual bytes read will be min(available_bytes, bufferSize).
     *                   Must be > 0 and match the actual buffer allocation.
     *
     * @return size_t The number of bytes actually read from the RX buffer.
     *                - **Data Available:** Returns 1 to `bufferSize` bytes
     *                - **No Data:** Returns 0 if the RX buffer is empty
     *                - **Error:** Some implementations may return 0 on error (indistinguishable from no data)
     *
     *                Return value tells you how many valid bytes are in `buffer`.
     *
     * @pre Start() must have been called successfully before this method.
     *      Calling Read() before Start() has undefined behavior.
     *
     * @post The bytes returned are removed from the RX buffer (consumed).
     *       Subsequent Read() calls will return any remaining bytes.
     *
     * @note **Non-Blocking:** The method returns immediately. If no data is available,
     *       it returns 0 without blocking. Polling loops should call Read() repeatedly
     *       or use event callbacks for data-arrival notification.
     *
     * @note **Circular Buffer:** HardwareSerial uses a circular RX buffer (typically 256 bytes).
     *       If data arrives faster than Read() is called, the buffer wraps and old data is lost.
     *       For reliable communication, call Read() frequently (usually > 100 times per second).
     *
     * @note **No Message Boundaries:** UART transmits a stream of bytes with no higher-level framing.
     *       A single Send() call may be split across multiple Read() calls, or multiple sends
     *       may be combined in one Read(). Higher layers must handle framing (e.g., delimiters,
     *       length prefixes, checksums).
     *
     * @note **Variable Byte Count:** The return value varies depending on when Read() is called
     *       relative to incoming data. You might receive 1 byte, then 10, then 0.
     *       Handle variable-length reads in your protocol decoder.
     *
     * @note **Thread-safety:** NOT thread-safe without external synchronization.
     *       Concurrent calls from multiple tasks cause race conditions on the RX buffer.
     *       Use a mutex to protect Read() calls if needed.
     *
     * @warning **Null Pointer:** Passing `buffer == nullptr` causes undefined behavior (crash).
     *          Always provide a valid buffer pointer.
     *
     * @warning **Buffer Overrun:** Passing `bufferSize` without allocating that much memory
     *          causes a buffer overrun and undefined behavior (memory corruption, crash).
     *          Allocate: `new uint8_t[bufferSize]`.
     *
     * @warning **Zero Buffer Size:** Passing `bufferSize == 0` causes undefined behavior.
     *          Always allocate at least 1 byte if you call Read().
     *
     * @warning **Data Loss on Overflow:** If data arrives faster than Read() consumes it,
     *          the RX buffer overflows and old data is silently dropped. No error is reported.
     *          Monitor the throughput and read frequency to avoid this.
     *
     * @warning **Port Not Started:** If Start() wasn't called or Stop() was called, Read() returns 0.
     *          Check IsConnected() before Read().
     *
     * @see Send() for transmitting data
     * @see IsConnected() to verify port is active
     */
    size_t Read(uint8_t* buffer, size_t bufferSize) override;

    /**
     * @brief Checks if the serial connection is active and initialized.
     * @details For `HardwareSerial`, this returns true if Start() was successfully called
     *          and Stop() has not been called subsequently. The method checks the internal
     *          `_started` flag, reflecting initialization state (not actual hardware health).
     *
     * @return true if the serial port is open, initialized, and ready for Send/Read.
     *         false if the port has not been started or has been stopped.
     *
     * @note **State Check Only:** This method checks the logical state flag, not the underlying
     *       hardware health. A sudden USB disconnect (for Serial) may not immediately update this flag.
     *
     * @note **Lightweight:** This method is very fast (just a boolean check) and safe to call frequently.
     *
     * @note **Real-Time Check:** For critical applications, call IsConnected() immediately before
     *       important Send/Read operations to catch races with Stop() calls.
     *
     * @warning **Not Hardware Health Check:** IsConnected() doesn't verify that the connected device
     *          is actually responding. A true return means "we initialized the port," not "the device is present."
     *
     * @warning **USB Disconnect Edge Case:** On USB serial (Serial), unplugging the cable
     *          may not immediately reflect in IsConnected(). The OS may take seconds to report
     *          the disconnection. For robustness, handle Send/Read failures gracefully.
     *
     * @see Start()
     * @see Stop()
     */
    bool IsConnected() override;

    /**
     * @brief Stops the serial communication.
     * @details Calls `end()` on the underlying HardwareSerial to close the port,
     *          disable the UART hardware, and release resources. After this call,
     *          Send() and Read() will fail (return 0) until Start() is called again.
     *
     * @post The serial port is closed:
     *       - UART hardware is disabled
     *       - TX and RX buffers are released
     *       - `IsConnected()` returns false
     *       - `Send()/Read()` fail silently (return 0)
     *
     * @note **Resource Release:** Stop() releases the UART hardware, making it available
     *       for other devices. Important for sharing hardware or power-saving.
     *
     * @note **Idempotency:** Calling Stop() multiple times is safe; subsequent calls do nothing.
     *
     * @note **Pending Data Loss:** Any data in RX buffer is lost. Sent data may still be
     *       transmitting via ISR. Ensure all important data is read/written before Stop().
     *
     * @note **Restart:** After Stop(), call Start() to re-initialize the port and resume operation.
     *
     * @note **Called by Destructor:** The destructor automatically calls Stop(), so explicit
     *       Stop() before object destruction is optional but recommended.
     *
     * @warning **Timing:** If Stop() is called while data is being transmitted (TX ISR active),
     *          the transmission may be cut off. Allow time for pending TX to complete if critical.
     *
     * @see Start()
     * @see ~ESP32Serial()
     */
    void Stop() override;

private:
    /**
     * @brief Constructs an ESP32Serial instance wrapping a HardwareSerial object.
     * @details Private constructor; use the Create() factory method instead.
     *
     * @param serial Reference to the HardwareSerial object to wrap (typically global Serial, Serial1, etc.).
     * @param baudRate The baud rate to configure (passed to HardwareSerial.begin()).
     *
     * @post The wrapper is initialized with references to the serial object and baud rate.
     *       Hardware is NOT yet initialized; call Start() to do so.
     */
    ESP32Serial(HardwareSerial& serial = Serial, uint32_t baudRate = 115200);

    /**
     * @brief Reference to the wrapped HardwareSerial object.
     * @note Global Arduino HardwareSerial objects (Serial, Serial1, Serial2) are statically allocated.
     * @note References are safer than pointers for this use case (guaranteed non-null).
     */
    HardwareSerial& _serial;
};

} // namespace Motion::Core::IO