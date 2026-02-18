/**
 * @file GenericMotor.h
 * @brief Defines the GenericMotor class, a hardware interface for various motor types.
 * @details Provides a standardized abstraction layer for different motor technologies (DC, stepper, servo).
 */

#pragma once

#include "Core/IO/Actuator/BaseActuator.h"

namespace Motion::Core::IO {

/**
 * @brief A generic hardware interface for different types of motors (e.g., DC, Brushless, Stepper).
 * @details This class provides a standardized interface for controlling motors by abstracting
 *          away the specifics of the underlying hardware. It inherits from `BaseActuator<float>`,
 *          expecting a floating-point command value (typically normalized from -1.0 to +1.0)
 *          to control motor speed and direction.
 *
 *          **Motor Types Supported:**
 *          - **DC Motors:** Simple brushed motors, bidirectional, speed controlled via PWM
 *          - **Stepper Motors:** Stepped rotation, position controlled via step pulses
 *          - **Servo Motors:** Angle controlled, position feedback built-in
 *          - **Brushless DC (BLDC):** Efficient, requires commutation (typically via ESC)
 *          - **Induction Motors:** AC motors, typically fixed speed with soft-start
 *
 *          **Command Interpretation:**
 *          The float command value has a conventional interpretation:
 *          - **Positive Values (0 to +1.0):** Forward/clockwise rotation at increasing speeds
 *            - 0.0: Stopped
 *            - 0.5: Half-speed forward
 *            - 1.0: Full-speed forward
 *          - **Negative Values (-1.0 to 0):** Reverse/counter-clockwise rotation
 *            - -0.5: Half-speed backward
 *            - -1.0: Full-speed backward
 *
 *          **Physical Realization:**
 *          The actual command implementation depends on the derived class:
 *          - **DC Motor:** PWM duty cycle on H-Bridge pins
 *          - **Servo Motor:** Pulse width modulation (servo protocol, typically 1-2 ms pulses)
 *          - **Stepper Motor:** Step frequency and direction line state
 *
 * @note **Template Specialization:** GenericMotor is a template class specializing BaseActuator<float>.
 *       All motors use float commands for consistency, even if the underlying hardware (e.g., stepper)
 *       has different physical control mechanisms.
 *
 * @note **Bidirectional Control:** The design assumes all motors can move in both directions.
 *       Unidirectional motors (e.g., BLDC ESC with throttle-only control) require wrapper logic.
 *
 * @note **Start/Stop Lifecycle:** Motors follow a standard lifecycle:
 *       1. Create (factory instantiation)
 *       2. Start() (hardware initialization)
 *       3. SetCommand() (repeated during operation)
 *       4. Stop() (cleanup and halt)
 *       5. Destructor (resource deallocation)
 *
 * @see BaseActuator<float> for the template base class
 * @see GenericDCMotor for DC motor-specific implementation
 * @see ESP32DCMotor for ESP32 hardware implementation
 */
class GenericMotor : public BaseActuator<float> {
protected:
    /**
     * @brief Constructs a new GenericMotor object.
     * @details Protected constructor; direct instantiation is not intended. The class serves as
     *          a base for derived motor implementations (ESP32DCMotor, GenericServoMotor, etc.).
     *
     *          Initialization chain:
     *          GenericMotor(name)
     *            → BaseActuator<float>(name)
     *            → IActuator(name, "float", sizeof(float))
     *
     * @param name A human-readable name for the motor, used for identification and debugging.
     *             Examples: "Left Motors", "Right Motor", "Shoulder Joint", "Gripper".
     *             Used in logging, telemetry, and error messages.
     *             **Critical:** The string must have static or long-lived storage duration.
     *             String literals are always safe; stack-allocated strings are unsafe.
     *
     * @post The motor is initialized with:
     *       - Name stored in IActuator._name
     *       - Type string set to "float" (via get_typename<float>())
     *       - Type size set to sizeof(float) = 4 bytes
     *       - Internal command state (_command) default-constructed (uninitialized)
     *       - Hardware resources NOT yet allocated (call Start())
     *
     * @note **Protected Access:** This constructor is protected, preventing direct instantiation.
     *       Derived concrete classes (ESP32DCMotor, etc.) call this in their constructors.
     *
     * @note **Type Deduction:** The type information (float) is automatically deduced from the
     *       template and embedded in the actuator metadata. This enables runtime type checking
     *       and serialization of command values.
     *
     * @note **String Lifetime:** The provided name pointer is stored directly (not copied).
     *       The string must remain valid throughout the motor's lifetime.
     *       Invalid scope:
     *       ```cpp
     *       {
     *           char motor_name[20] = "my_motor";
     *           auto motor = ESP32DCMotor::Create(motor_name, config);  // UNSAFE!
     *       }  // motor_name goes out of scope → dangling pointer
     *       motor->GetName();  // CRASH!
     *       ```
     *       Valid scope:
     *       ```cpp
     *       auto motor = ESP32DCMotor::Create("my_motor", config);  // String literal → safe
     *       motor->GetName();  // OK, literal persists in program text
     *       ```
     *
     * @warning The name pointer is stored by reference. Ensure the string scope extends
     *          beyond the motor's lifetime. Use string literals whenever possible.
     *
     * @warning Avoid passing stack-allocated or dynamically-allocated strings with limited scope.
     *          If necessary, use std::string or equivalent to manage lifetime:
     *          ```cpp
     *          std::string name = std::string("Motor") + std::to_string(id);
     *          // Careful: lifetime management needed
     *          ```
     */
    explicit GenericMotor(const char* name) : BaseActuator<float>(name) {}
};

/**
 * @relates GenericMotor
 * @brief Smart handle for Motor instances.
 * @details Uses smart pointer for automatic lifetime management.
 *          Recommended for ownership and passing motors through the application.
 *
 * @note Enables multiple subsystems to reference the same motor safely,
 *       with automatic cleanup when the last reference is destroyed.
 */
using GenericMotorHandle = ActuatorPointer(GenericMotor);

} // namespace Motion::Core::IO