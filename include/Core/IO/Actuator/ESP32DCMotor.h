/**
 * @file ESP32DCMotor.h
 * @brief ESP32 Hardware motor interface implementation.
 * @details Defines the ESP32-specific implementation for controlling a DC motor.
 */

#pragma once

#include "Core/IO/Actuator/GenericDCMotor.h"
#include "Arduino.h"

namespace Motion::Core::IO {

/**
 * @brief An ESP32 hardware interface for a DC motor.
 * @details This class implements the GenericDCMotor interface using ESP32-specific hardware features
 *          (e.g., LEDC PWM) to control motor speed and direction.
 */
class ESP32DCMotor : public GenericDCMotor {
public:
    /**
     * @brief Constructs a new ESP32DCMotor object.
     * @details Initializes the motor instance with a name and hardware configuration.
     * @param name A human-readable name for the motor, used for identification and logging.
     * @param config The configuration structure containing DC motor command pins and settings.
     * @note Ensure that the GPIO pins specified in `config` are valid and available on the ESP32.
     */
    explicit ESP32DCMotor(const char* name, DCMotorConfig config);

    /**
     * @brief Destroys the ESP32DCMotor object.
     * @details Ensures that the motor is stopped and releases any hardware resources associated with it.
     */
    ~ESP32DCMotor();

    /**
     * @brief Initializes and starts the DC motor hardware interface.
     * @details Configures the GPIO pins and PWM channels required for motor operation.
     * @return true if the hardware was successfully initialized; false otherwise.
     */
    bool Start() override;

    /**
     * @brief Sends a control command to the DC motor.
     * @details Translates the abstract command value into hardware-specific signals (e.g., PWM duty cycle).
     * @param command The control value to apply to the motor. The interpretation of this value
     *                (e.g., normalized -1.0 to 1.0, or voltage) depends on the base class contract.
     * @warning Ensure `Start()` has been called successfully before sending commands.
     */
    void SendCommand(float command) override;

    /**
     * @brief Stops the DC motor immediately.
     * @details Sets the motor output to neutral/zero, effectively halting rotation.
     */
    void Stop() override;
};

} // namespace Motion::Core::IO