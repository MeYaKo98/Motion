/**
 * @file ISensor.h
 * @brief Defines the non-template base interface for generic sensor management.
 * @details This file contains the abstract base class that all sensors must implement.
 *          It provides common metadata and lifecycle management methods.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <memory>

namespace Motion::Core::IO {

#ifdef DOXYGEN
#define SensorPointer(T) T*;
#else
#define SensorPointer(T) std::shared_ptr<T>;
#endif

/**
 * @brief Non-template base interface for generic sensor management.
 * @details This abstract class provides a uniform interface for interacting with various sensors.
 *          It handles metadata storage (name, data type description, data size) and defines
 *          the lifecycle methods (Start, Stop) and data retrieval method (GetSerialisedReading).
 *          Derived template class BaseSensor<T> specializes this for specific sensor types.
 *
 *          **Typical Sensor Types:**
 *          - Encoder: returns int32_t (tick count)
 *          - Accelerometer: returns float (acceleration value)
 *          - Temperature: returns float (temperature value)
 *          - Distance: returns uint16_t (distance in mm)
 *          - Pressure: returns float (pressure in Pa)
 *
 * @note **Template Specialization:** While this is the non-template base, derived classes
 *       use templates to enforce type safety between the sensor reading type and the implementation.
 *
 * @see BaseSensor for the template-based implementation
 */
class ISensor {
public:
    /**
     * @brief Returns the human-readable name of the sensor.
     * @details The name is set during construction and is used for logging, telemetry, and debugging.
     *
     * @return const char* Pointer to a null-terminated string containing the sensor's name.
     *                     Examples: "Left Encoder", "Gyroscope", "Ultrasonic Distance".
     *
     * @note The returned pointer is valid for the lifetime of the sensor instance.
     * @note This method is const and provides read-only access (thread-safe).
     */
    const char* GetName() const { return _name; }

    /**
     * @brief Returns the readable string describing the data type of sensor readings.
     * @details Provides a human-readable representation of the C++ type used for readings.
     *          Useful for telemetry, logging, and type verification in higher-layer code.
     *
     * @return const char* Pointer to a null-terminated string describing the data type.
     *                     Examples: "int32_t", "float", "uint16_t".
     *                     The exact format depends on the type introspection implementation.
     *
     * @note Typically generated via compile-time type demangling (`get_typename<T>()`).
     * @note For debugging: can be compared against expected types to detect misconfiguration.
     */
    const char* GetDataType() const { return _dataType; }

    /**
     * @brief Returns the size of a single sensor reading in bytes.
     * @details Useful for serialization, buffer allocation, memory layout calculations.
     *          Equals `sizeof(T)` where T is the reading type template parameter.
     *
     * @return size_t The number of bytes required to store a single reading.
     *                Examples: 4 for float, 4 for int32_t, 2 for int16_t.
     *
     * @note This value is invariant—it never changes during the lifetime of the sensor.
     * @note Used by GetSerialisedReading() to determine memcpy size.
     */
    size_t GetTypeSize() const { return _typeSize; }

    /**
     * @brief Starts the sensor hardware interface.
     * @details Initializes hardware resources, configures pins, and performs any other
     *          necessary setup to make the sensor operational. This must be called before
     *          reading data via GetReading().
     *
     *          Implementation details vary by sensor type:
     *          - **Encoder:** Initialize GPIO pins, configure PCNT unit, attach ISR
     *          - **I2C Sensor:** Initialize bus, configure settings, verify communication
     *          - **Analog Sensor:** Configure ADC channel, set sampling rate
     *          - **Serial Sensor:** Open port, configure baud rate
     *
     * @return true if the sensor started successfully and is ready to provide readings.
     * @return false if initialization failed (e.g., hardware unavailable, I2C NAK).
     *
     * @post On success, the sensor is ready to receive read requests via GetReading().
     * @post On failure, the sensor remains non-operational; reading will return invalid data.
     *
     * @note **Idempotency:** Calling Start() twice without Stop() may have platform-specific behavior.
     *       Document idempotency requirements in derived classes.
     *
     * @note **Resource Allocation:** Always pair Start() with Stop() to avoid resource leaks,
     *       especially for encoders and I2C/SPI sensors that hold hardware resources.
     *
     * @warning **Prerequisites:** Before calling Start(), ensure:
     *          - Required GPIO pins are not in use elsewhere
     *          - External hardware (sensors, pull-up resistors) are properly connected
     *          - I2C/SPI buses are properly configured
     *          - Power supply is stable (important for analog sensors)
     *
     * @see Stop()
     * @see GetReading()
     */
    virtual bool Start() = 0;

    /**
     * @brief Stops the sensor hardware interface.
     * @details Disables hardware resources, releases pins, and puts the sensor into a safe state.
     *          After calling this method, the sensor is no longer operational and must be restarted
     *          via Start() before accepting new read requests.
     *
     *          Derived implementations typically:
     *          - Disable interrupt handlers (for sensors like encoders)
     *          - Release GPIO resources
     *          - Close communication channels (I2C, SPI, UART)
     *          - Disable power-saving features may be moot
     *
     * @return void
     *
     * @post The sensor is in a stopped, safe state.
     * @post Hardware resources are released and available for other uses.
     * @post Subsequent calls to GetReading() will either fail silently or return stale data.
     *
     * @note **Resource Cleanup:** Essential for sensors that hold scarce resources
     *       (e.g., encoders use PCNT units, I2C uses bus pins).
     *
     * @note **Idempotency:** Calling Stop() multiple times is safe—subsequent calls do nothing.
     *
     * @note **Power Efficiency:** Stopping unused sensors reduces power consumption,
     *       important for battery-powered robots.
     *
     * @see Start()
     */
    virtual void Stop() = 0;

    /**
     * @brief Retrieves the latest sensor reading in serialized binary form.
     * @details Converts the internal sensor state (of the template type T) into a byte sequence.
     *          This is essential for telemetry, logging, and transmitting sensor data over
     *          communication channels.
     *
     * @param[out] buffer Pointer to the output buffer where the serialized reading will be copied.
     *                    The buffer is allocated and managed by the caller.
     *
     * @return void
     *
     * @pre The `buffer` pointer must point to valid, allocated memory.
     * @post The binary representation of the latest sensor reading is written to `buffer`.
     *
     * @warning **Buffer Safety:** The caller must ensure that `buffer` is allocated with at least
     *          `GetTypeSize()` bytes. Providing a smaller buffer causes **buffer overflow** and
     *          **undefined behavior**. Always verify: `buffer_size >= GetTypeSize()`.
     *
     * @note **Memory Layout:** Uses `memcpy` for serialization, so byte order and alignment
     *       follow the platform's native representation (typically little-endian on x86/ARM).
     *       For cross-platform data exchange, consider byte-order conversion.
     *
     * @note **Stale Data:** The returned value is the most recent reading cached by GetReading().
     *       If GetReading() hasn't been called recently, the data may be stale.
     *
     * @note **Null Pointer Handling:** Some implementations may silently ignore nullptr;
     *       others may crash. Always provide a valid buffer pointer.
     *
     * @see BaseSensor::GetSerialisedReading()
     * @see GetReading()
     */
    virtual void GetSerialisedReading(uint8_t* buffer) = 0;

    /**
     * @brief Virtual destructor for polymorphic cleanup.
     * @details Ensures that derived class destructors are properly called when
     *          the sensor is deleted through a base class pointer.
     */
    virtual ~ISensor() = default;

protected:
    /**
     * @brief Constructs a new ISensor with metadata.
     * @details Initializes the non-template base class with sensor metadata.
     *          Derived template classes call this constructor to set up metadata.
     *
     * @param name Human-readable identifier for the sensor (e.g., "Left Encoder").
     *             The string is not copied; only a pointer is stored.
     *             Must have static or long-lived scope.
     * @param dataType String description of the reading type (e.g., "int32_t").
     *               Typically generated via `get_typename<T>()`.
     * @param typeSize Size of the reading type in bytes (e.g., `sizeof(T)`).
     *
     * @note **String Ownership:** The caller retains ownership of name and dataType strings.
     *       Use string literals for safety: they have static lifetime.
     *
     * @warning If stack-allocated strings are passed, behavior becomes undefined when strings
     *          go out of scope. Dangling pointers will cause crashes when accessed.
     */
    ISensor(const char* name, const char* dataType, size_t typeSize)
        : _name(name), _dataType(dataType), _typeSize(typeSize) {}

    /**
     * @brief Human-readable name of the sensor.
     * @details Used for identification, logging, telemetry, and debugging.
     * @note This is a pointer to a string; the string is not owned by this class.
     */
    const char* _name;

    /**
     * @brief String description of the reading data type.
     * @details Used for telemetry and type verification.
     * @note This is a pointer to a string; the string is not owned by this class.
     */
    const char* _dataType;

    /**
     * @brief Size of the reading data type in bytes.
     * @details Used for serialization buffer allocation and memcpy operations.
     */
    size_t _typeSize;
};

} // namespace Motion::Core::IO