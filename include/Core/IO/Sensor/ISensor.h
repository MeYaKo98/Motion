/**
 * @file ISensor.h
 * @brief Defines the non-template base interface for generic sensor management.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace Motion::Core::IO {

/**
 * @brief Non-template base interface for generic sensor management.
 * @details This abstract class provides a uniform interface for interacting with various sensors.
 *          It handles metadata storage (name, data type description, data size) and defines
 *          the lifecycle methods (Start, Stop) and data retrieval method (GetSerialisedReading).
 */
class ISensor {
protected:
    /** @brief The human-readable name of the sensor. */
    const char* _name;
    /** @brief The human-readable description of the data type. */
    const char* _dataType;
    /** @brief The size of the data type in bytes. */
    size_t _typeSize;

public:
    /**
     * @brief Constructs a new ISensor object.
     * @param name A human-readable sensor name.
     * @param dataType A human-readable description of the sensor data type.
     * @param typeSize The size of the sensor data type in bytes.
     * @warning **Pointer Lifetime:** The `name` and `dataType` pointers are stored directly.
     *          The caller must ensure these strings remain valid for the lifetime of the ISensor object.
     *          Passing pointers to temporary strings or stack variables that go out of scope will result in undefined behavior.
     */
    explicit ISensor(const char* name, const char* dataType, size_t typeSize) : _name(name), _dataType(dataType), _typeSize(typeSize) {}

    /**
     * @brief Returns the name of the sensor.
     * @return The human-readable name of the sensor as a C-string.
     */
    const char* GetName() const { return _name; }

    /**
     * @brief Returns the readable string for the data type read by the sensor.
     * @return The human-readable data type description as a C-string.
     */
    const char* GetDataType() const { return _dataType; }

    /**
     * @brief Returns the size of the sensor data in bytes.
     * @return The size of the data type in bytes.
     */
    size_t GetTypeSize() const { return _typeSize; }

    /**
     * @brief Starts the sensor hardware interface.
     * @details Initializes and activates the sensor. This must be called before attempting to read data.
     * @return true if the sensor started successfully; false otherwise.
     */
    virtual bool Start() = 0;

    /**
     * @brief Retrieves a serialized reading from the sensor.
     * @details Writes the raw byte representation of the sensor's current reading into the provided buffer.
     * @param buffer Output buffer for serialized data.
     * @warning **Buffer Safety:** The provided `buffer` must be allocated with a size of at least `_typeSize` bytes.
     *          Providing a smaller buffer or a null pointer will result in undefined behavior (e.g., buffer overflow or crash).
     */
    virtual void GetSerialisedReading(uint8_t* buffer) = 0;

    /**
     * @brief Stops the sensor hardware interface.
     * @details Deactivates the sensor and releases any hardware resources if necessary.
     */
    virtual void Stop() = 0;

    /**
     * @brief Virtual destructor.
     */
    virtual ~ISensor() = default;

    /**
     * @brief Deleted copy constructor.
     * @details Sensors represent physical hardware and are non-copyable.
     */
    ISensor(const ISensor&) = delete;            // No copying

    /**
     * @brief Deleted assignment operator.
     * @details Sensors represent physical hardware and are non-assignable.
     */
    ISensor& operator=(const ISensor&) = delete; // No assignment
};

} // namespace Motion::Core::IO