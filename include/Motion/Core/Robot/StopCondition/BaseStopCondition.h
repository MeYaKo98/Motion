/**
 * @file BaseStopCondition.h
 * @brief Defines the abstract base interface for robot stop conditions.
 * @details Stop conditions are strategies for determining when a motion is "complete."
 *          They answer the question: "Should the robot stop moving now?"
 */

#pragma once

#include <memory>

namespace Motion::Core::Robot {

#define StopConditionPointer(T) std::shared_ptr<T>

/**
 * @brief Abstract base class defining the interface for motion stop conditions.
 * @details This interface allows different stopping criteria (e.g., time-based, error-based,
 *          sensor-based) to be used interchangeably by the motion control system.
 *          Derived classes must implement the `ShouldExit()` and `Reset()` methods.
 *
 *          Stop conditions are typically embedded in control loops like this:
 *          ```cpp
 *          stopCondition->Reset();
 *          while (!stopCondition->ShouldExit(error)) {
 *              // Perform control iteration
 *              // Update error based on current state
 *          }
 *          ```
 *
 *          **Common Stop Condition Implementations:**
 *          - **ToleranceStopCondition:** Stops when error < tolerance threshold
 *          - **TimeoutStopCondition:** Stops after a maximum time has elapsed
 *          - **AndStopCondition:** Stops when all sub-conditions are satisfied (logical AND)
 *          - **OrStopCondition:** Stops when any sub-condition is satisfied (logical OR)
 *          - **WindowStopCondition:** Stops when error stays below threshold for N iterations
 *
 * @note **Composite Pattern:** The framework supports combining stop conditions using
 *       logical operators (&&, ||) to create complex stopping logic.
 *
 * @see ToleranceStopCondition for a simple tolerance-based implementation
 * @see AndStopCondition for combining multiple conditions with AND logic
 * @see OrStopCondition for combining multiple conditions with OR logic
 */
class BaseStopCondition;

using BaseStopConditionHandle = StopConditionPointer(BaseStopCondition);

class BaseStopCondition{
public:
    /**
     * @brief Virtual destructor.
     * @details Ensures that the destructor of the derived class is called when
     *          an object is deleted through a pointer to the base class.
     *          This is essential for proper cleanup of any resources allocated by the derived class.
     *
     * @note All virtual classes must have virtual destructors to support polymorphic behavior.
     */
    virtual ~BaseStopCondition() = default;

    /**
     * @brief Determines if the motion should stop based on the current error or state.
     * @details This method is the core of the stop condition logic. It evaluates whether
     *          the stopping criterion has been met and should return true when the motion should cease.
     *          This method is typically called in a high-frequency control loop.
     *
     * @param error The current error value, whose interpretation depends on the specific implementation:
     *              - **Position Error:** absolute difference between target and actual position
     *              - **Velocity Error:** difference between target and actual velocity
     *              - **Angle Error:** rotational error in radians
     *              - **Generic Error:** application-specific error metric
     *              The sign and magnitude are significant. Document the error calculation in derived implementations.
     *
     * @return true if the stop condition is satisfied and the motion should cease immediately.
     *         false if the condition is not yet satisfied and motion should continue.
     *
     * @post If this method returns true, the caller should cease motion commands and consider
     *       the movement complete.
     *
     * @pre The condition should have been reset via `Reset()` before the motion loop begins.
     *
     * @note **Frequency:** This method is typically called in a high-frequency loop (100-1000 Hz).
     *       It should be **non-blocking** and execute in microseconds. Avoid I/O, logging, or
     *       dynamic memory allocation.
     *
     * @note **Hysteresis/Debouncing:** Some implementations may use internal counters to prevent
     *       triggering on transient noise. For example, requiring the error to stay in the
     *       acceptable range for multiple consecutive calls before signaling success.
     *
     * @note **Stateful:** This method may maintain internal state (e.g., counters, time) between calls.
     *       Always call `Reset()` before starting a new motion sequence to clear stale state.
     *
     * @note **Error Interpretation:** The error value is typically computed by the caller based
     *       on the specific motion being controlled. Document what error metric is expected
     *       at the derived class level.
     *
     * @note **Default Behavior:** For basic conditions, the method should be simple and deterministic:
     *       given the same error, it should return the same result (ignoring state changes).
     *
     * @warning **Race Conditions:** If this method is called from multiple threads without synchronization,
     *          concurrent access to internal state may cause undefined behavior.
     *          Use external locking if multi-threaded access is possible.
     *
     * @warning **State Dependency:** The return value may depend on previous calls and internal state.
     *          Always reset the condition before reusing it for a new motion sequence.
     *
     * @warning **Error Sign:** Ensure the error sign convention matches the implementation.
     *          An inverted error sign may cause the condition to behave unexpectedly.
     *
     * @see Reset()
     * @see ToleranceStopCondition for a simple tolerance-based example
     */
    virtual bool ShouldExit(float error) = 0;

    /**
     * @brief Resets the internal state of the stop condition.
     * @details This method should be called before starting a new motion to ensure
     *          that any internal counters, timers, or accumulated state are cleared.
     *          This is critical for correct behavior when reusing the same stop condition instance.
     *
     *          State that should be reset includes:
     *          - Iteration counters (for debouncing or windowing)
     *          - Time references (start time for timeout conditions)
     *          - Flags (e.g., "first call" indicator)
     *          - Accumulated error or history buffers
     *
     * @post After calling this method, the stop condition should be in a "fresh" state,
     *       ready to evaluate a new motion sequence as if it were just constructed.
     *
     * @post The first call to `ShouldExit()` after `Reset()` will be evaluated based on
     *       the current error and the condition's parameters, without influence from previous motions.
     *
     * @pre This method **must** be called before starting each new motion loop.
     *      Failure to reset will cause the condition to return incorrect results.
     *
     * @note **Timing:** Call this method at the very beginning of a new motion command,
     *       before entering the control loop.
     *
     * @note **Idempotent:** Calling `Reset()` multiple times without intervening calls
     *       to `ShouldExit()` should have the same effect as calling it once.
     *
     * @warning **State Corruption:** If `ShouldExit()` is called without a preceding `Reset()`,
     *          the condition may evaluate with stale state from the previous motion, causing
     *          incorrect behavior (e.g., exiting too early).
     *
     * @warning **Non-Idempotent Implementations:** Some implementations may not be fully idempotent
     *          if they rely on timers or external state. Document any side effects of `Reset()`.
     *
     * @see ShouldExit()
     */
    virtual void Reset() = 0;

protected:
    /**
     * @brief Constructs a new Base Stop Condition object.
     * @details Provides a default constructor for derived classes. The stop condition
     *          is in an uninitialized state and should be reset before use.
     */
    BaseStopCondition() {};

};

using BaseStopConditionHandle = StopConditionPointer(BaseStopCondition);

} // namespace Motion::Core::Robot