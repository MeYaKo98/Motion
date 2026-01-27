/**
 * @file BaseStopCondition.h
 * @brief Defines the abstract base interface for robot stop conditions.
 */

#pragma once

namespace Motion::Core::Robot {

/**
 * @brief Abstract base class defining the interface for motion stop conditions.
 * @details This interface allows different stopping criteria (e.g., time-based, error-based,
 *          sensor-based) to be used interchangeably by the motion control system.
 *          Derived classes must implement the `ShouldExit` and `Reset` methods.
 */
class BaseStopCondition{
public:
    /**
     * @brief Constructs a new Base Stop Condition object.
     */
    BaseStopCondition() {};

    /**
     * @brief Virtual destructor.
     * @details Ensures that the destructor of the derived class is called when
     *          an object is deleted through a pointer to the base class.
     */
    virtual ~BaseStopCondition() = default;

    /**
     * @brief Determines if the motion should stop based on the current error or state.
     * @param error The current error value (e.g., position error, velocity error).
     *              The interpretation of this value depends on the specific implementation.
     * @note This method is typically called within a high-frequency control loop and should be non-blocking.
     * @return true if the stop condition is satisfied and the motion should cease; false otherwise.
     */
    virtual bool ShouldExit(float error) = 0;

    /**
     * @brief Resets the internal state of the stop condition.
     * @details This method should be called before starting a new motion to ensure
     *          that any internal counters, timers, or accumulated state are cleared.
     * @note Failure to reset may cause the condition to trigger prematurely in subsequent motions.
     */
    virtual void Reset() = 0;
};

} // namespace Motion::Core::Robot