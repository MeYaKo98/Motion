/**
 * @file IActuator.h
 * @brief A non-template base interface for all hardware actuators in the system.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <Core/Diagnostics/Logger.h>

namespace Motion::Core::IO {

/**
 * @brief Non-template base interface for generic actuator management.
 * @details This abstract class defines the contract for all hardware actuators.
 *          It provides metadata storage (name, data type description, size) and
 *          pure virtual methods for lifecycle management (Start, Stop) and data retrieval.
 */
class IActuator {
protected:
    /** @brief The human-readable name of the actuator. */
    const char* _name;
    /** @brief The string representation of the data type used to command the actuator. */
    const char* _dataType;
    /** @brief The size of the command data in bytes. */
    size_t _typeSize;
    
public:
    /**
     * @brief Constructs a new IActuator object.
     * @param name A human-readable actuator name.
     * @param dataType A human-readable string describing the actuator data type.
     * @param typeSize The size of the command data type in bytes.
     * @note The `name` and `dataType` pointers are stored directly. The caller must ensure
     *       these strings remain valid for the lifetime of this object (typically string literals).
     */
    explicit IActuator(const char* name, const char* dataType, size_t typeSize) : _name(name), _dataType(dataType), _typeSize(typeSize) {
        if (_name == nullptr) LOG_ERROR("Actuator name must cannot be NULL");
    }

    /**
     * @brief Returns the name of the actuator.
     * @return const char* The name of the actuator.
     */
    const char* GetName() const { return _name; }

    /**
     * @brief Returns the readable string for the data type to command the actuator.
     * @return const char* The data type description string.
     */
    const char* GetDataType() const { return _dataType; }

    /**
     * @brief Returns the command data size.
     * @return size_t The size of the command data in bytes.
     */
    size_t GetTypeSize() const { return _typeSize; }

    /**
     * @brief Starts the actuator hardware interface.
     * @return true If the actuator started successfully.
     * @return false If the actuator failed to start.
     */
    virtual bool Start() = 0;

    /**
     * @brief Retrieves the last serialized command sent to the actuator.
     * @param buffer Output buffer where the serialized data will be written.
     * @warning The caller is responsible for ensuring that the `buffer` is allocated
     *          with a size of at least `GetTypeSize()` bytes to prevent buffer overflows.
     */
    virtual void GetSerialisedCommand(uint8_t* buffer) = 0;

    /**
     * @brief Stop the actuator hardware interface.
     * @details This method should safely shut down the actuator, ensuring it is in a safe state.
     */
    virtual void Stop() = 0;

    /**
     * @brief Virtual destructor.
     */
    virtual ~IActuator() = default;

    /**
     * @brief Deleted copy constructor to prevent copying of actuator instances.
     */
    IActuator(const IActuator&) = delete;

    /**
     * @brief Deleted assignment operator to prevent assignment of actuator instances.
     */
    IActuator& operator=(const IActuator&) = delete;
};

} // namespace Motion::Core::IO