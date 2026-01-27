/**
 * @file AndStopCondition.h
 * @brief Defines a stop condition that aggregates multiple stop conditions using a logical AND.
 */

#pragma once

#include "Core/Robot/StopCondition/BaseStopCondition.h"
#include <vector>

namespace Motion::Core::Robot {

/**
 * @brief A composite stop condition that is met only when all of its contained conditions are met.
 * @details This class implements the Composite design pattern for stop conditions. It allows
 *          treating a group of stop conditions as a single logical unit. The `ShouldExit` method will
 *          return true only if `ShouldExit` returns true for **all** the contained conditions.
 * @see BaseStopCondition
 */
class AndStopCondition : public BaseStopCondition{
public:
    /**
     * @brief Constructs an AndStopCondition with a variable number of stop conditions.
     * @tparam Args Variadic template arguments. Must be pointers to `BaseStopCondition` or derived classes.
     * @param args A comma-separated list of raw pointers to stop condition objects.
     * @warning **Memory Management:** This class stores raw pointers. The caller is responsible for managing the lifecycle
     *          of these objects and ensuring they remain valid for the lifetime of the AndStopCondition instance.
     */
    template<typename... Args>
    explicit AndStopCondition(Args... args) {
        _conditions.reserve(sizeof...(args));
        int dummy[] = { 0, (_conditions.push_back(args), 0)... };
        (void)dummy;
    }

    /**
     * @brief Destroys the AndStopCondition object.
     * @note **No Deallocation:** This destructor does not free the memory of the contained stop condition pointers.
     *       The owner of those pointers is responsible for their cleanup to prevent memory leaks.
     */
    ~AndStopCondition();

    /**
     * @brief Determines if the robot should stop by evaluating all contained stop conditions.
     * @param error The current error value, passed down to each contained stop condition.
     * @return true if **all** contained stop conditions are satisfied; false if **any** condition is not satisfied.
     */
    bool ShouldExit(float error) override;

    /**
     * @brief Resets the internal state of all contained stop conditions.
     * @details Iterates through the list of stop conditions and calls `Reset()` on each one.
     *          Ensures the composite condition is ready for a new motion sequence.
     */
    void Reset() override;

private:
    /**
     * @brief A collection of raw pointers to the aggregated stop conditions.
     */
    std::vector<BaseStopCondition*> _conditions;
};

} // namespace Motion::Core::Robot