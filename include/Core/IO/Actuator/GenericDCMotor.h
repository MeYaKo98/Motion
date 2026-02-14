/**
 * @file GenericDCMotor.h
 * @brief Hardware motor interface for DC motors.
 * @details Defines the configuration and interface for controlling a generic DC motor
 *          using a dual-pin configuration.
 */

#pragma once

#include "Core/IO/Actuator/GenericMotor.h"

namespace Motion::Core::IO {

/**
 * @brief Configuration structure for a DC Motor.
 * @details Holds the hardware pin assignments required to drive the motor.
 */
struct DCMotorConfig {
    /** @brief The first control pin for the motor (e.g., IN1). */
    uint8_t pinA;
    /** @brief The second control pin for the motor (e.g., IN2). */
    uint8_t pinB;
};

/**
 * @brief A Generic Motor hardware interface for DC motors.
 * @details This class implements the GenericMotor interface specifically for DC motors
 *          that are controlled via two pins (typically connected to an H-Bridge).
 */
class GenericDCMotor : public GenericMotor {
public:
    /**
     * @brief Constructs a new Generic DC Motor object.
     * @details Initializes the motor with a name and hardware configuration.
     * @param name A human-readable name for the motor.
     * @param config The configuration structure containing the DC motor command pins.
     * @note This constructor initializes the base GenericMotor with the provided name.
     */
    explicit GenericDCMotor(const char* name, DCMotorConfig config) : GenericMotor(name), _config(config) {};

protected:
    /**
     * @brief The hardware configuration for this motor.
     */
    DCMotorConfig _config;
};

} // namespace Motion::Core::IO