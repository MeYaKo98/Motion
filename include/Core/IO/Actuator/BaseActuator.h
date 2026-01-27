/**
 * @file BaseActuator.h
 * @brief Base interface for all hardware actuators in the system.
 */

#pragma once

#include "Core\IO\Actuator\IActuator.h"
#include <string.h>

namespace Motion::Core::IO {

/**
 * @brief A template base class for actuator management.
 * @details Provides a common interface for setting commands and retrieving serialized data for actuators.
 * @tparam T The data type of the command sent to the actuator.
 */
template <typename T>
class BaseActuator : public IActuator{
public:
    /**
     * @brief Constructs a new BaseActuator object.
     * @param name A human-readable name for the actuator.
     * @note The actuator type is initialized to "tobeupdated" and the data size is set to sizeof(T).*
     * @todo implement auto type name
     */
    explicit BaseActuator(const char* name) : IActuator(name, "tobeupdated", sizeof(T)) {}

    /**
     * @brief Sets and sends a command to the actuator.
     * @details Stores the command internally and dispatches it via the pure virtual SendCommand method.
     * @param command The command value to send.
     */
    void SetCommand(T command)
    {
        _command = command;
        SendCommand(command);
    }


    /**
     * @brief Retrieves the last command sent to the actuator in serialized form.
     * @param[out] buffer The output buffer where the serialized command will be copied.
     * @warning The buffer must be at least `sizeof(T)` bytes large. Providing a smaller buffer will result in a buffer overflow.
     * @note If `buffer` is nullptr, the operation is silently ignored.
     */
    virtual void GetSerialisedCommand(uint8_t* buffer) override
    {
        if (!buffer) return;
        memcpy(buffer, &_command, sizeof(T));
    }

    /**
     * @brief Virtual destructor.
     */
    virtual ~BaseActuator() = default;

    /**
     * @brief Copy constructor is deleted to prevent copying of hardware actuator instances.
     */
    BaseActuator(const BaseActuator&) = delete;

    /**
     * @brief Assignment operator is deleted to prevent assignment of hardware actuator instances.
     */
    BaseActuator& operator=(const BaseActuator&) = delete;

protected:
    /**
     * @brief Sends the command to the physical actuator.
     * @details This pure virtual function must be implemented by derived classes to handle the specific hardware communication.
     * @param command The command to send.
     */
    virtual void SendCommand(T command) = 0;
    
    /**
     * @brief Stores the last command sent to the actuator.
     */
    T _command;
};

} // namespace Motion::Core::IO