/**
 * @file GenericDCMotor.h
 * @brief Hardware motor interface for DC motors with dual-pin control.
 * @details Defines configuration and interface for controlling generic DC motors
 *          using H-Bridge or relay-based dual-pin motor drivers.
 */

#pragma once

#include "Core/IO/Actuator/GenericMotor.h"

namespace Motion::Core::IO {

/**
 * @brief Configuration structure for a DC Motor.
 * @details Holds the hardware pin assignments required to drive a DC motor.
 *          A DC motor requires a motor driver (H-Bridge or relay module) with two control pins:
 *          one for forward direction and one for reverse direction.
 *
 *          **Typical Motor Driver Configurations:**
 *          - **H-Bridge (L298N, DRV8833, etc.):**
 *            - pinA: IN1 (forward control, PWM)
 *            - pinB: IN2 (reverse control, PWM)
 *            - Speed controlled by duty cycle on active pin
 *            - Direction by selecting which pin is active
 *
 * @note **Pin Independence:** pinA and pinB must be different GPIO pins.
 *       Many motor drivers support H-Bridge logic but not independent PWM on both pins
 *       simultaneously. Consult your driver's datasheet.
 *
 * @see ESP32DCMotor for an ESP32-specific implementation
 */
struct DCMotorConfig {
    /**
     * @brief The first control pin for the motor (e.g., IN1, Forward relay).
     * @details Used to control forward motion or the primary direction when PWM is applied.
     *          Must be a valid GPIO pin number for the target platform.
     *          Typical range: 0-39 on ESP32, 0-13 on Arduino Uno, etc.
     *
     * @note **Do NOT set to same pin as pinB.** The factory will reject this configuration.
     *
     * @note **Purpose:** Typically controls forward motion when set HIGH or receives PWM.
     *       On H-Bridges, sets one side of the bridge.
     */
    uint8_t pinA;

    /**
     * @brief The second control pin for the motor (e.g., IN2, Reverse relay).
     * @details Used to control reverse motion or the secondary direction when PWM is applied.
     *          Must be a valid GPIO pin number for the target platform.
     *          Must be different from pinA.
     *
     * @note **Do NOT set to same pin as pinA.** The factory will reject this configuration.
     *
     * @note **Purpose:** Typically controls reverse motion when set HIGH or receives PWM.
     *       On H-Bridges, sets the opposite side of the bridge from pinA.
     */
    uint8_t pinB;
};

/**
 * @brief A Generic Motor hardware interface for DC motors.
 * @details This class implements the GenericMotor interface specifically for DC motors
 *          that are controlled via two pins (typically connected to an H-Bridge driver).
 *          It is a base class for platform-specific implementations (e.g., ESP32DCMotor).
 *
 *          **DC Motor Basics:**
 *          - Brushed DC motors are simple: apply +V to start forward, -V to start backward
 *          - Require external H-Bridge or relay driver to control direction
 *          - PWM can control speed via duty cycle
 *          - Simple, proven technology widely used in robotics
 *
 *          **Control Method:**
 *          Two pins allow independent control of forward and reverse:
 *          - pinA active, pinB inactive → forward motion
 *          - pinB active, pinA inactive → reverse motion
 *          - Both inactive → coast/stop
 *          - Both active → depends on driver (may brake or short)
 *
 *          **Typical Speed Range:**
 *          - Command value -1.0 to +1.0 (normalized)
 *          - Translates to PWM duty cycle 0-255 or similar
 *          - Actual motor speed depends on voltage, motor load, battery state
 *
 * @note **Template Hierarchy:** Inherits from GenericMotor<float>, which specifies that
 *       command values are floating-point (typically -1.0 to +1.0).
 *
 * @note **Non-Template Base:** This class is non-template because DC motors always use float
 *       command values. Template specialization isn't beneficial.
 *
 * @see GenericMotor for the template-based motor abstraction
 * @see BaseActuator for the low-level actuator interface
 * @see ESP32DCMotor for an ESP32-specific implementation
 */
class GenericDCMotor : public GenericMotor {
protected:
    /**
     * @brief Constructs a new Generic DC Motor object.
     * @details Protected constructor; derived classes or factories instantiate concrete implementations.
     *          Initializes the motor with a name and hardware configuration.
     *
     * @param name A human-readable name for the motor (e.g., "Left Motor", "Wheel Drive").
     *             Used for identification, logging, and telemetry.
     *             String must be statically allocated or have long-lived scope
     *             (e.g., string literals are safe).
     *
     * @param config The configuration structure containing the DC motor command pins:
     *               - config.pinA: First control pin (forward)
     *               - config.pinB: Second control pin (reverse)
     *               Stored internally for later use by derived implementations.
     *
     * @post The motor instance is initialized with:
     *       - Name stored via GenericMotor (inherited)
     *       - Configuration stored in `_config` member
     *       - Base actuator state initialized
     *       - Hardware is NOT yet initialized (call Start() when ready)
     *
     * @note **Initialization Order:** This constructor calls GenericMotor(name), which
     *       in turn calls BaseActuator(name), which calls IActuator(name, type, size).
     *       The type is derived from the template parameter (float) and size is sizeof(float).
     *
     * @note **Protected Constructor:** Prevents direct instantiation. Derived concrete classes
     *       (ESP32DCMotor, etc.) use this constructor to initialize their base class.
     *
     * @warning The `name` pointer is not copied; only a reference is stored.
     *          Ensure the name string remains valid for the motor's lifetime.
     *          String literals are always safe; stack-allocated strings are not.
     *
     * @warning The `config` reference may be stored. Ensure it remains valid
     *          (or the configuration is copied by derived classes).
     */
    explicit GenericDCMotor(const char* name, DCMotorConfig config) : GenericMotor(name), _config(config) {};

    /**
     * @brief The hardware configuration for this motor.
     * @details Stores the GPIO pin assignments (pinA, pinB) for motor control.
     *          Used by derived implementations (e.g., ESP32DCMotor) to configure
     *          and drive the actual GPIO pins.
     *
     * @note **Read-Only:** This member should not be modified after construction.
     *
     * @note **Platform-Specific:** The pin numbers are platform-specific
     *       (e.g., 0-39 on ESP32, 0-13 on Arduino Uno). The derived implementation
     *       must validate and apply these pins to the correct platform.
     */
    DCMotorConfig _config;
};

/**
 * @relates GenericDCMotor
 * @brief Smart handle for DC Motor instances.
 * @details Uses shared_ptr for automatic lifetime management.
 *          Recommended for all motor ownership and passing through the application.
 */
using GenericDCMotorHandle = ActuatorPointer(GenericDCMotor);

} // namespace Motion::Core::IO