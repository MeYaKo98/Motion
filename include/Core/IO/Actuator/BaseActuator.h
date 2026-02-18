/**
 * @file BaseActuator.h
 * @brief Base template class for all hardware actuators in the Motion Framework.
 * @details Provides a common interface and command storage for setting and retrieving actuator
 *          commands. This class serializes commands to byte buffers for telemetry and communication.
 */

#pragma once

#include "Core/IO/Actuator/IActuator.h"
#include "Core/IO/typeNameUtil.h"
#include <string.h>

namespace Motion::Core::IO {

/**
 * @brief A template base class for actuator management and command serialization.
 * @details Provides a common interface for setting commands and retrieving serialized data for actuators.
 *          Derived classes must implement the `SendCommand()` pure virtual method to handle hardware-specific
 *          communication. This class handles the storage and serialization of the most recent command.
 * @tparam T The data type of the command sent to the actuator (e.g., float for speed, int for PWM).
 */
template <typename T>
class BaseActuator : public IActuator{
public:
    /**
     * @brief Sets and sends a command to the actuator.
     * @details Stores the command value internally in `_command` and immediately dispatches it to the
     *          hardware via the pure virtual `SendCommand()` method. The stored command persists and can be
     *          retrieved later via `GetSerialisedCommand()`.
     * @param command The command value to send to the actuator.
     * @note The command is stored as a copy; the caller retains ownership of the input parameter.
     * @note Thread-safety: This function is **not** thread-safe. If called from multiple threads,
     *       the caller must synchronize access using appropriate locking mechanisms.
     */
    void SetCommand(T command)
    {
        _command = command;
        SendCommand(command);
    }


    /**
     * @brief Retrieves the last command sent to the actuator in serialized binary form.
     * @details Converts the internal `_command` member (of type T) into a byte sequence using `memcpy`.
     *          This is useful for telemetry, logging, and state synchronization across distributed systems.
     * @param[out] buffer The output buffer where the serialized command will be copied.
     *                    The buffer is expected to be allocated by the caller.
     * @return void
     * @warning **Buffer Safety:** The caller must ensure that `buffer` is allocated with at least `sizeof(T)` bytes.
     *          Providing a smaller buffer will result in a **buffer overflow** and undefined behavior.
     *          Always verify: `buffer != nullptr` and `buffer_size >= sizeof(T)`.
     * @note **Null Pointer Handling:** If `buffer` is nullptr, this method returns immediately without
     *       performing any operation. No exception is thrown; the call is silently ignored.
     * @note **Thread-safety:** This function is **not** thread-safe with respect to concurrent `SetCommand()` calls.
     *       If concurrent access is required, the caller must implement proper synchronization.
     * @see SetCommand()
     */
    virtual void GetSerialisedCommand(uint8_t* buffer) override
    {
        if (!buffer) return;
        memcpy(buffer, &_command, sizeof(T));
    }

    /**
     * @brief Virtual destructor.
     * @details Allows proper cleanup of derived class instances when deleted through a base class pointer.
     *          Ensures that any resources allocated by derived classes are properly deallocated.
     */
    virtual ~BaseActuator() = default;

    /**
     * @brief Copy constructor is deleted to prevent copying of hardware actuator instances.
     * @details Actuators represent hardware resources with internal state. Copying them would create
     *          aliasing issues and potential resource leaks. Move construction is also implicitly deleted.
     */
    BaseActuator(const BaseActuator&) = delete;

    /**
     * @brief Assignment operator is deleted to prevent assignment of hardware actuator instances.
     * @details Prevents unintended resource duplication and state corruption from assignment operations.
     */
    BaseActuator& operator=(const BaseActuator&) = delete;

protected:
    /**
     * @brief Constructs a new BaseActuator object.
     * @details Initializes the actuator with a human-readable name and configures the type and data size
     *          based on the template parameter T. The actuator type is automatically resolved from the
     *          template type using compile-time type introspection via `get_typename<T>()`.
     * @param name A human-readable unique identifier for the actuator (e.g., "Left Motor", "Gripper").
     *             This string is used for logging, telemetry, and debugging. The string must remain valid
     *             for the lifetime of this object (pointers are stored, not copies).
     * @note **Memory Ownership:** The name string is not copied; only a pointer is stored. Ensure the
     *       string has static lifetime (e.g., string literals are safe) or is managed elsewhere.
     * @note **Type Initialization:** The actuator type is set to the demangled C++ type name of T via
     *       `get_typename<T>()`, and the internal data size is automatically set to `sizeof(T)`.
     * @warning If the provided name pointer becomes invalid (e.g., stack-allocated string goes out of scope),
     *          accessing it later will result in undefined behavior.
     */
    explicit BaseActuator(const char* name) : IActuator(name, get_typename<T>(), sizeof(T)) {}
    
    /**
     * @brief Sends the command to the physical actuator hardware.
     * @details This pure virtual function must be implemented by derived classes to handle the specific
     *          hardware communication protocol. It is called synchronously from `SetCommand()`, so derived
     *          class implementations should prioritize responsiveness.
     * @param command The command value to transmit to the hardware.
     * @return void
     * @note **Implementation Requirement:** Derived classes **must** implement this method.
     *       Failing to do so will result in a linker error.
     * @note **Thread-safety:** The actual thread-safety of hardware transmission is determined by the
     *       derived class implementation. Document thread-safety guarantees at the derived class level.
     * @warning **Blocking I/O:** If the derived class performs blocking I/O (e.g., blocking serial write),
     *          consider the impact on real-time control loops. Use non-blocking alternatives or offload
     *          to a separate task/thread if latency is critical.
     */
    virtual void SendCommand(T command) = 0;
    
    /**
     * @brief Stores the last command sent to the actuator.
     * @details This member maintains the most recent command state, allowing `GetSerialisedCommand()`
     *          to serialize it without direct hardware access. Used for telemetry and external state queries.
     * @note **Initialization:** This member is default-constructed when the BaseActuator is created.
     *       For POD types, this means it contains uninitialized data until the first `SetCommand()` call.
     * @warning For complex types (non-POD), ensure the copy constructor and assignment operator are
     *          well-defined to avoid slicing or partial copies.
     */
    T _command;
};

template <typename T> using BaseActuatorHandle = ActuatorPointer(BaseActuator<T>);

} // namespace Motion::Core::IO