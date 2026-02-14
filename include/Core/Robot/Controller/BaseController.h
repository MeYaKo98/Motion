/**
 * @file BaseController.h
 * @brief Defines the base interface for closed-loop actuator controllers.
 * @details This interface standardizes the interaction with various control algorithms used to generate actuator commands based on error feedback.
 */

#pragma once

#include "Core/Diagnostics/Logger.h"

namespace Motion::Core::Robot {

/**
 * @brief Abstract base class for closed-loop actuator controllers.
 * @details This class provides a common interface for generating control commands based on error input.
 *          Concrete implementations must implement the `GenerateCommand` and `Reset` methods.
 */
class BaseController {
public:
    /**
     * @brief Constructs a new BaseController with specified output limits.
     * @param maxCommand The maximum allowable output command value.
     * @param minCommand The minimum allowable output command value.
     * @note The concrete implementation is responsible for enforcing these limits within `GenerateCommand`.
     */
    BaseController(float maxCommand, float minCommand) : _maxCommand(maxCommand), _minCommand(minCommand)  {
        if (_maxCommand < _minCommand) LOG_ERROR ("Min and Max values are invalid"); 
    }

    /**
     * @brief Virtual destructor for the BaseController.
     */
    virtual ~BaseController() = default;

    /**
     * @brief Resets the internal state of the controller.
     * @details This method should be called before starting a new control sequence to clear any accumulated state
     *          (e.g., integral windup in a PID controller).
     */
    virtual void Reset() = 0;

    /**
     * @brief Generates a control command based on the provided error.
     * @param error The difference between the setpoint and the actual value (Setpoint - Actual).
     * @return The computed control command.
     * @note Implementations should ensure the return value is clamped between `_minCommand` and `_maxCommand`.
     */
    virtual float GenerateCommand(float error) = 0;

protected:
    /** @brief The maximum allowable output command. */
    float _maxCommand;
    /** @brief The minimum allowable output command. */
    float _minCommand;
};

} // namespace Motion::Core::Robot