/**
 * @file TolerenceStopCondition.h
 * @brief Defines a stop condition based on an error tolerance threshold.
 */

#pragma once

#include "Core/Robot/StopCondition/BaseStopCondition.h"
#include "Core/Diagnostics/Logger.h"
#include <cmath>

namespace Motion::Core::Robot {

/**
 * @brief A stop condition that is satisfied when the error is within a specified tolerance.
 * @details This class checks if the magnitude of the provided error is less than or equal to
 *          the configured tolerance value. It is commonly used to determine if a motion
 *          has reached its target within an acceptable margin of error.
 */
class TolerenceStopCondition : public BaseStopCondition{
public:
    /**
     * @brief Constructs a new Tolerance Stop Condition.
     * @param tolerence The threshold value for the stop condition. The condition is met
     *                  if the absolute error is less than this value.
     */
    explicit TolerenceStopCondition(float tolerence);

    /**
     * @brief Destroys the Tolerance Stop Condition object.
     */
    ~TolerenceStopCondition();

    /**
     * @brief Determines if the robot should stop based on the current error.
     * @param error The current error value (e.g., target position - current position).
     * @return true if the absolute value of `error` is less than or equal to the tolerance; false otherwise.
     */
    bool ShouldExit(float error) override;

    /**
     * @brief Resets the internal state of the stop condition.
     * @details For this specific condition, this ensures compatibility with the
     *          `BaseStopCondition` interface, resetting any transient state if applicable.
     */
    void Reset() override;

protected:
    /**
     * @brief The configured error tolerance threshold.
     */
    float _tolerence;
};

} // namespace Motion::Core::Robot