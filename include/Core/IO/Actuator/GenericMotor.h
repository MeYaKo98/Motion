/**
 * @file GenericMotor.h
 * @brief Defines the GenericMotor class, a hardware interface for various motor types.
 */

#pragma once

#include "Core\IO\Actuator\BaseActuator.h"

namespace Motion::Core::IO {

/**
 * @brief A generic hardware interface for different types of motors (e.g., DC, Brushless).
 * @details This class provides a standardized interface for controlling motors by abstracting
 *          away the specifics of the underlying hardware. It inherits from `BaseActuator<float>`,
 *          expecting a floating-point value (typically from -1.0 to 1.0) to control motor
 *          speed and direction.
 */
class GenericMotor : public BaseActuator<float> {
public:
    /**
     * @brief Constructs a new GenericMotor object.
     * @param name A human-readable name for the motor, used for identification and debugging.
     * @note The provided name string must have a lifetime that extends beyond this object's
     *       destruction, as only a pointer to it is stored. String literals are safe.
     */
    explicit GenericMotor(const char* name) : BaseActuator<float>(name) {}
};

} // namespace Motion::Core::IO