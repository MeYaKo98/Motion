/**
 * @file BaseSensor.h
 * @brief Base template class for all hardware sensors in the Motion Framework.
 * @details Provides a standard interface for sensor reading and serialization. Derived classes implement
 *          hardware-specific reading logic via the `ReadSensor()` pure virtual method.
 */

#pragma once

#include "Core/IO/Sensor/ISensor.h"
#include "Core/IO/typeNameUtil.h"
#include <string.h>

namespace Motion::Core::IO {

/**
 * @brief A template base class for sensor management and data retrieval.
 * @details This class provides a standard interface for sensors, handling common tasks
 *          such as storing the latest reading and serializing it for telemetry. Concrete sensor implementations
 *          must inherit from this class and implement the pure virtual `ReadSensor()` method.
 * @tparam T The data type of the sensor reading (e.g., int32_t for encoder ticks, float for analog voltage).
 *          Must be a type compatible with `memcpy` serialization (typically POD types).
 */
template <typename T>
class BaseSensor: public ISensor {
public:
    /**
     * @brief Retrieves the latest reading from the sensor.
     * @details This method triggers a hardware read by calling the pure virtual `ReadSensor()` method,
     *          updates the internal stored reading in `_reading`, and returns the value. This reading
     *          persists and can be accessed via `GetSerialisedReading()` even after the method returns.
     * @return T The current sensor reading from the hardware.
     * @note The reading is of the same type as the template parameter T. Callers should understand the
     *       data type and its range (e.g., int32_t for encoder ticks, float for voltage).
     * @note **Thread-safety:** This method is **not** thread-safe. Concurrent calls to `GetReading()` and
     *       `GetSerialisedReading()` may return inconsistent state. If needed, synchronize access with a mutex.
     * @warning **Blocking I/O:** If the derived class's `ReadSensor()` performs blocking hardware I/O
     *          (e.g., blocking SPI read), this method will block the caller. Avoid calling from
     *          time-critical loops; consider offloading to a dedicated sensor task.
     */
    virtual T GetReading()
    {
        _reading = ReadSensor();
        return _reading;
    }

    /**
     * @brief Serializes the current sensor reading into a byte buffer.
     * @details Converts the internal `_reading` member (of type T) into a byte sequence using `memcpy`.
     *          This is essential for telemetry over serial/network and for inter-process communication.
     * @param[out] buffer Pointer to the destination buffer where the serialized data will be copied.
     *                    The buffer is expected to be allocated by the caller.
     * @return void
     * @warning **Buffer Safety:** The caller must ensure that `buffer` is allocated with at least `sizeof(T)` bytes.
     *          Providing a smaller buffer will result in a **buffer overflow** and **undefined behavior**.
     *          Always verify buffer size before calling: `buffer != nullptr` and `buffer_size >= sizeof(T)`.
     * @note **Null Pointer Handling:** If `buffer` is nullptr, the function returns immediately without
     *       performing any operation. No exception is thrown; the call is silently ignored.
     *       **Design Pattern:** This is a fail-safe pattern to prevent crashes from null pointers.
     * @note **Thread-safety:** This function is **not** thread-safe with respect to concurrent `GetReading()` calls.
     *       If concurrent access is required, synchronize externally or use atomic types for `_reading`.
     * @note **Data Consistency:** The serialized data reflects the value of `_reading` at the time of the
     *       `memcpy` operation. If `_reading` is modified concurrently, the result is undefined.
     * @see GetReading()
     */    
    void GetSerialisedReading(uint8_t* buffer) override
    {
        if (!buffer) return;
        memcpy(buffer, &_reading, sizeof(T));
    }
    
protected:
    /**
     * @brief Constructs a new BaseSensor object.
     * @details Initializes the sensor with a human-readable name and configures metadata based on the
     *          template parameter T. The sensor type is automatically resolved using compile-time type introspection.
     * @param name A human-readable unique identifier for the sensor (e.g., "Left Encoder", "Gyroscope").
     *             Used for logging, telemetry, and debugging. The string is not copied; only a pointer
     *             is stored, so it must remain valid for the sensor's lifetime.
     * @note **Memory Ownership:** The name string is not copied. Use string literals or ensure the string
     *       has static lifetime. If a stack-allocated string is passed, accessing it later causes undefined behavior.
     * @note **Type Initialization:** The sensor type is set to the demangled C++ type name of T via
     *       `get_typename<T>()`, and the internal data size is set to `sizeof(T)`.
     * @warning **Null String Danger:** Passing a null pointer as `name` will cause crashes when the name
     *          is accessed internally (e.g., in logging).
     */
    explicit BaseSensor(const char* name) : ISensor(name, get_typename<T>(), sizeof(T)) {}

    /**
     * @brief Reads the raw value from the hardware sensor.
     * @details This pure virtual function must be implemented by derived classes to interface
     *          with the specific sensor hardware. It is called from `GetReading()` to obtain the latest value.
     * @return T The current reading from the sensor hardware.
     * @note **Implementation Requirement:** Derived classes **must** implement this method.
     *       Failing to do so results in a linker error.
     * @note **Blocking Operations:** This method may perform blocking I/O. Document any blocking behavior
     *       in the derived class implementation.
     * @note **Error Handling:** If the sensor hardware fails, return a default/safe value (e.g., 0).
     *       Document failure behavior in the derived class.
     * @warning **Thread-safety:** The thread-safety of hardware reads depends on the derived class.
     *          Document any thread-safety guarantees provided by the implementation.
     */
    virtual T ReadSensor() = 0;

    /**
     * @brief Stores the most recent sensor reading.
     * @details This member holds the last value retrieved from `ReadSensor()`. It persists until
     *          `GetReading()` is called again, allowing multiple accesses without re-reading hardware.
     * @note **Initialization:** This member is default-constructed when the sensor is created.
     *       For POD types, this means uninitialized data until the first `GetReading()` call.
     * @note **Concurrency:** For thread-safe reads, consider using `std::atomic<T>` in derived classes,
     *       though serialization via `memcpy` requires special handling for atomic types.
     * @warning **Type Requirements:** T must be a type that `memcpy` can safely handle (typically POD).
     *          Non-POD types may cause issues with serialization.
     */
    T _reading;
};

/**
 * @relates BaseSensor
 * @brief Smart handle (shared pointer) for Sensor instances.
 * @details Manages the lifetime of sensor objects, ensuring proper cleanup when no longer referenced.
 */
template <typename T> using BaseSensorHandle = SensorPointer(BaseSensor<T>);

} // namespace Motion::Core::IO