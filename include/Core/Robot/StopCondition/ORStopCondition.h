/**
 * @file ORStopCondition.h
 * @brief Defines a stop condition that aggregates multiple stop conditions using a logical OR.
 */

#pragma once

#include "Core/Robot/StopCondition/BaseStopCondition.h"
#include <vector>

namespace Motion::Core::Robot {

/**
 * @brief A composite stop condition that is met when **any** of its contained conditions are met.
 * @details This class implements the Composite design pattern for stop conditions. It allows
 *          treating a group of stop conditions as a single logical unit. The `ShouldExit` method will
 *          return true if `ShouldExit` returns true for **any** of the contained conditions.
 *          This is useful for combining multiple safety checks where any single failure should trigger a stop.
 * @see BaseStopCondition
 */
class ORStopCondition : public BaseStopCondition{
public:
    /**
     * @brief Destroys the ORStopCondition object.
     * @note **No Deallocation:** This destructor does not free the memory of the contained stop condition pointers.
     *       The owner of those pointers is responsible for their cleanup to prevent memory leaks.
     */
    ~ORStopCondition();

    /**
     * @brief Determines if the robot should stop by evaluating the contained stop conditions.
     * @details This method uses short-circuit evaluation; it returns `true` immediately if any condition is met.
     * @param error The current error value, passed down to each contained stop condition.
     * @return true if **any** contained stop condition is satisfied; false if **none** of the conditions are satisfied.
     */
    bool ShouldExit(float error) override;

    /**
     * @brief Resets the internal state of all contained stop conditions.
     * @details Iterates through the list of stop conditions and calls `Reset()` on each one.
     *          Ensures the composite condition is ready for a new motion sequence.
     */
    void Reset() override;

    friend BaseStopConditionHandle operator||(BaseStopConditionHandle conditionA, BaseStopConditionHandle conditionB);

protected:
    explicit ORStopCondition(BaseStopConditionHandle conditionA, BaseStopConditionHandle conditionB);

    BaseStopConditionHandle _conditionsA;

    BaseStopConditionHandle _conditionsB;
};

BaseStopConditionHandle operator||(BaseStopConditionHandle conditionA, BaseStopConditionHandle conditionB);

} // namespace Motion::Core::Robot