/**
 * @file IActuator.h
 * @brief Non-template base interface for all hardware actuators in the system.
 * @details This file defines the abstract interface that all actuators must implement.
 *          It provides common metadata retrieval and lifecycle management methods.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <memory>
#include <Core/Diagnostics/Logger.h>

namespace Motion::Core::IO {

#ifdef DOXYGEN
#define ActuatorPointer(T) T*
#else
#define ActuatorPointer(T) std::shared_ptr<T>
#endif

/**
 * @brief Non-template base interface for generic actuator management.
 * @details This abstract class defines the contract for all hardware actuators in the system.
 *          It provides metadata storage (name, data type description, size) and
 *          pure virtual methods for lifecycle management (Start, Stop) and data retrieval.
 *          Derived template class BaseActuator<T> specializes this interface for specific command types.
 *
 *          **Typical Actuator Types:**
 *          - DC Motors: commanded with a float value (-1.0 to 1.0)
 *          - Servo Motors: commanded with an angle (0 to 180 degrees)
 *          - Stepper Motors: commanded with step count
 *          - Linear Actuators: commanded with velocity or position
 *
 * @note **Type Safety:** While this base class is non-template, derived classes use templates
 *       to enforce type safety between the command type and the hardware implementation.
 *
 * @see BaseActuator for the template-based implementation
 */
class IActuator {
public:
    /**
     * @brief Returns the human-readable name of the actuator.
     * @details The name is typically set during construction and is used for identification,
     *          logging, telemetry, and debugging purposes.
     * @return const char* Pointer to a null-terminated string containing the actuator's name.
     *                     Example: "Left Motor", "Gripper", "Servo1".
     * @note The returned pointer is valid for the lifetime of the actuator instance.
     * @note This method is const and thread-safe (read-only access).
     */
    const char* GetName() const { return _name; }

    /**
     * @brief Returns the readable string description of the command data type.
     * @details Provides a human-readable representation of the C++ type used to command this actuator.
     *          This is useful for telemetry, logging, and type verification in higher-layer code.
     *
     * @return const char* Pointer to a null-terminated string describing the data type.
     *                     Examples: "float", "int32_t", "uint8_t".
     *                     The exact format depends on the type introspection implementation.
     *
     * @note This typically uses compile-time type demangling via `get_typename<T>()`.
     * @note For debugging: can be compared against expected types to detect misconfiguration.
     */
    const char* GetDataType() const { return _dataType; }

    /**
     * @brief Returns the size of the command data type in bytes.
     * @details Useful for serialization, buffer allocation, and memory layout calculations.
     *          Equals `sizeof(T)` where T is the command type template parameter.
     *
     * @return size_t The number of bytes required to store a single command value.
     *                Examples: 4 for float, 1 for uint8_t, 2 for int16_t.
     *
     * @note Invariant: This value never changes during the lifetime of the actuator.
     * @note Used by GetSerialisedCommand() to determine the size of memcpy operation.
     */
    size_t GetTypeSize() const { return _typeSize; }

    /**
     * @brief Starts the actuator hardware interface.
     * @details Initializes hardware resources, configures pins, and performs any other
     *          necessary setup to make the actuator operational. This must be called before
     *          sending commands via SetCommand().
     *
     *          Implementation details vary by actuator type:
     *          - **DC Motor:** Configure GPIO pins as OUTPUT, initialize PWM
     *          - **Servo Motor:** Attach servo to pin, set default position
     *          - **Stepper Motor:** Configure step/direction pins, set step rate
     *          - **Linear Actuator:** Calibrate sensors, set home position
     *
     * @return true if the actuator started successfully and is ready to receive commands.
     * @return false if initialization failed (e.g., GPIO conflict, hardware not available).
     *
     * @post On success, the actuator is ready to receive commands via SetCommand().
     *       On failure, the actuator remains non-operational; calling SetCommand() is undefined.
     *
     * @note **Idempotency:** Calling Start() twice without Stop() may have platform-specific behavior.
     *       Some implementations may silently succeed; others may fail or reinitialize.
     *       Document idempotency in derived classes.
     *
     * @note **Resource Allocation:** Always pair Start() with Stop() to avoid resource leaks.
     *
     * @warning **Prerequisites:** Before calling Start(), ensure:
     *          - All required GPIO pins are not in use by other parts of the application
     *          - Hardware is properly powered and initialized
     *          - Peripheral clocks are enabled (typically automatic on modern boards)
     *
     * @see Stop()
     * @see SetCommand()
     */
    virtual bool Start() = 0;

    /**
     * @brief Stops the actuator hardware interface.
     * @details Disables hardware resources, releases pins, and puts the actuator into a safe state.
     *          After calling this method, the actuator is no longer operational and must be restarted
     *          via Start() before accepting new commands.
     *
     *          Derived implementations typically:
     *          - Stop any ongoing motion (e.g., set motor speed to 0)
     *          - Release GPIO resources
     *          - Disable PWM or other peripherals
     *          - Save state if needed
     *
     * @return void
     *
     * @post The actuator is in a safe, stopped state (e.g., motor not moving).
     * @post Resources are released and available for other applications.
     * @post Subsequent calls to SetCommand() will have no effect until Start() is called.
     *
     * @note **Safety:** This method should be called to ensure safe shutdown, especially for
     *       power-consuming or potentially dangerous actuators (e.g., gripper, drill).
     *
     * @note **Idempotency:** Calling Stop() multiple times is safe—subsequent calls do nothing.
     *
     * @warning **Data Loss:** Any pending commands or state changes may not be saved.
     *          Ensure important data is persisted before calling Stop().
     *
     * @see Start()
     */
    virtual void Stop() = 0;

    /**
     * @brief Retrieves the last command sent to the actuator in serialized binary form.
     * @details Converts the internal command state (of the template type T) into a byte sequence.
     *          This is essential for telemetry, logging, and state synchronization across
     *          distributed systems.
     *
     * @param[out] buffer Pointer to the output buffer where the serialized command will be copied.
     *                    The buffer is allocated and managed by the caller.
     *
     * @return void
     *
     * @pre The `buffer` pointer must point to valid, allocated memory.
     * @post The binary representation of the last command value is written to `buffer`.
     *
     * @warning **Buffer Safety:** The caller must ensure that `buffer` is allocated with at least
     *          `GetTypeSize()` bytes. Providing a smaller buffer results in a **buffer overflow**
     *          and **undefined behavior**. Always verify: `buffer_size >= GetTypeSize()`.
     *
     * @note **Memory Layout:** The serialization uses `memcpy`, so byte order and alignment
     *       follow the platform's native representation (typically little-endian on x86/ARM).
     *       For cross-platform compatibility, consider additional byte-order handling.
     *
     * @note **Null Pointer Handling:** Some implementations may check for nullptr and return
     *       gracefully; others may crash. Always ensure buffer is valid.
     *
     * @see BaseActuator::GetSerialisedCommand()
     * @see SetCommand()
     */
    virtual void GetSerialisedCommand(uint8_t* buffer) = 0;

    /**
     * @brief Virtual destructor for polymorphic cleanup.
     * @details Ensures that derived class destructors are properly called when
     *          the actuator is deleted through a base class pointer.
     */
    virtual ~IActuator() = default;

protected:
    /**
     * @brief Constructs a new IActuator with metadata.
     * @details Initializes the non-template base class with common metadata.
     *          Derived template classes call this constructor to set up metadata.
     *
     * @param name Human-readable identifier for the actuator (e.g., "Left Motor").
     *             The string is not copied; only a pointer is stored.
     *             The string must remain valid for the lifetime of the actuator.
     * @param dataType String description of the command type (e.g., "float").
     *                 Typically generated via `get_typename<T>()`.
     * @param typeSize Size of the command type in bytes (e.g., `sizeof(T)`).
     *
     * @note **String Ownership:** The caller retains ownership of the name and dataType strings.
     *       Use string literals for safety: they have static lifetime.
     *
     * @warning If stack-allocated strings are passed, the behavior becomes undefined when the
     *          strings go out of scope. The dangling pointers will cause crashes when accessed.
     */
    IActuator(const char* name, const char* dataType, size_t typeSize)
        : _name(name), _dataType(dataType), _typeSize(typeSize) {}

    /**
     * @brief Human-readable name of the actuator.
     * @details Used for identification, logging, telemetry.
     * @note This is a pointer to a string; the string is not owned by this class.
     */
    const char* _name;

    /**
     * @brief String description of the command data type.
     * @details Used for telemetry and type verification.
     * @note This is a pointer to a string; the string is not owned by this class.
     */
    const char* _dataType;

    /**
     * @brief Size of the command data type in bytes.
     * @details Used for serialization buffer allocation and memcpy operations.
     */
    size_t _typeSize;
};

} // namespace Motion::Core::IO