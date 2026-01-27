/**
 * @file BaseSensor.h
 * @brief Base interface for all hardware sensors in the system.
 */

#pragma once

#include "Core\IO\Sensor\ISensor.h"
#include <string.h>

namespace Motion::Core::IO {

/**
 * @brief A template base class for sensor management and data retrieval.
 * @tparam T The data type of the sensor reading (e.g., int, float, struct).
 * @details This class provides a standard interface for sensors, handling common tasks
 *          such as storing the latest reading and serializing it. Concrete sensor implementations
 *          must inherit from this class and implement the `ReadSensor()` method.
 */
template <typename T>
class BaseSensor : public ISensor {
public:
    /**
     * @brief Constructs a new BaseSensor object.
     * @param name A human-readable unique identifier for the sensor (e.g., "Left Encoder").
     * @todo implement auto type name
     */
    explicit BaseSensor(const char* name) : ISensor(name, "tobeupdated", sizeof(T)) {}

    /**
     * @brief Retrieves the latest reading from the sensor.
     * @details This method triggers a hardware read by calling the pure virtual `ReadSensor()` method,
     *          updates the internal stored reading, and returns the value.
     * @return T The current sensor reading.
     */
    virtual T GetReading()
    {
        _reading = ReadSensor();
        return _reading;
    }

    /**
     * @brief Serializes the current sensor reading into a byte buffer.
     * @param buffer Output buffer where the serialized data will be copied.
     * @warning **Buffer Safety:** The caller must ensure that `buffer` is allocated with at least `sizeof(T)` bytes.
     *          Failure to do so will result in a buffer overflow.
     * @note If `buffer` is nullptr, the function returns immediately without performing any operation.
     */    
    void GetSerialisedReading(uint8_t* buffer) override
    {
        if (!buffer) return;
        memcpy(buffer, &_reading, sizeof(T));
    }
    
protected:
    /**
     * @brief Reads the raw value from the hardware sensor.
     * @details This pure virtual function must be implemented by derived classes to interface
     *          with the specific sensor hardware.
     * @return T The current reading from the sensor.
     */
    virtual T ReadSensor() = 0;

    T _reading;
};

} // namespace Motion::Core::IO