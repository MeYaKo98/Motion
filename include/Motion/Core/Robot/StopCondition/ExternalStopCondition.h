/**
 * @file ExternalStopCondition.h
 * @brief Defines a stop condition controlled by an external signal.
 * @details This stop condition allows external logic or user control to determine when motion should stop,
 *          independent of error metrics or elapsed time.
 */

#pragma once

#include "Motion/Core/Robot/StopCondition/BaseStopCondition.h"
#include "Motion/Core/Logger.h"
#include <atomic>

namespace Motion::Core::Robot {

/**
 * @brief A stop condition controlled by an external signal or user decision.
 * @details This class implements a stop condition that is not determined by the error metric,
 *          but rather by an external signal set via the SetFlag() method. The condition is
 *          thread-safe through the use of atomic booleans.
 *
 *          **Use Cases:**
 *          - Emergency stop: External safety system signals an immediate halt
 *          - User-initiated stop: User presses a stop button mid-motion
 *          - State machine transitions: Higher-level control logic triggers a stop
 *          - Conditional execution: Stop based on non-error sensor inputs (collision detection, limit switches)
 *          - Testing and debugging: Manually trigger stop conditions without error metrics
 *
 *          **Behavioral Characteristics:**
 *          - The ShouldExit() method ignores the error parameter completely
 *          - The decision to stop depends solely on the internal flag state
 *          - The flag is atomic, enabling thread-safe updates from external sources
 *          - Reset() clears the flag, allowing the condition to be reused
 *
 *          **Thread Safety:**
 *          This condition is designed for thread-safe operation. The internal flag is an
 *          `std::atomic<bool>`, allowing safe concurrent reads and writes without explicit locking.
 *          Example: One thread can update the flag via SetFlag() while another thread calls ShouldExit().
 *
 * @note **Error Independence:** Unlike ToleranceStopCondition, this condition does not care about
 *       the error value passed to ShouldExit(). The error parameter is accepted for API compatibility
 *       but is completely ignored.
 *
 * @note **Composite Conditions:** This condition is especially useful in composite conditions.
 *       For example, combine with ToleranceStopCondition using OrStopCondition to implement
 *       "stop if error is low OR emergency signal is received".
 *
 * @warning **Default State:** The condition starts in the "do not exit" state (flag = false).
 *          Remember to call SetFlag(true) when you want the motion to stop.
 *
 * @see BaseStopCondition for the base interface
 * @see ToleranceStopCondition for error-based stopping logic
 * @see OrStopCondition for combining with other conditions using OR logic
 * @see AndStopCondition for combining with other conditions using AND logic
 */
class ExternalStopCondition;

using ExternalStopConditionHandle = StopConditionPointer(ExternalStopCondition);

class ExternalStopCondition : public BaseStopCondition {
public:

    /**
     * @brief Factory method to create an ExternalStopCondition instance.
     * @details Creates a new externally-controlled stop condition initialized to the "not exiting" state.
     *          The condition will not trigger a stop until SetFlag(true) is explicitly called.
     *
     * @return ExternalStopConditionHandle A shared pointer to the newly created condition.
     *
     * @post The internal flag is initialized to false.
     * @post The condition is ready to be used in motion control loops.
     *
     * @note **Usage Pattern:**
     *       @code
     *       auto stopCondition = ExternalStopCondition::Create();
     *       stopCondition->Reset();
     *       
     *       // Later, when external signal indicates stop:
     *       stopCondition->SetFlag(true);
     *       @endcode
     *
     * @see SetFlag()
     */
    static ExternalStopConditionHandle Create();

    /**
     * @brief Destroys the External Stop Condition object.
     * @details Performs cleanup. For this simple condition, no special resource cleanup is needed.
     */
    ~ExternalStopCondition();

    /**
     * @brief Determines if the robot should stop based on the external flag.
     * @details Returns the current state of the internal flag, completely ignoring the error parameter.
     *          This allows an external source to control the stop decision independent of motion metrics.
     *
     * @param error The current error value. This parameter is **completely ignored** by this condition.
     *              It is accepted for API compatibility with BaseStopCondition but has no effect
     *              on the result.
     *
     * @return true if the internal flag has been set to true via SetFlag(true);
     *         false otherwise (default state or after Reset()).
     *
     * @note **Thread-Safe:** This method uses atomic load semantics to safely read the flag
     *       from concurrent threads without blocking.
     *
     * @note **Performance:** This function is O(1) and extremely fast (atomic load).
     *       Suitable for high-frequency control loops (1000+ Hz).
     *
     * @note **Error Ignored:** The error parameter is completely ignored. You may pass any value,
     *       usually 0 or NaN, without affecting the behavior.
     *
     * @note **Deterministic:** Given the same internal flag state, this method always returns
     *       the same result, making it predictable and suitable for safety-critical applications.
     *
     * @warning **No State Transition Logic:** The condition does not automatically transition.
     *          Once SetFlag(true) is called, ShouldExit() will return true until Reset() is called.
     *          There is no hysteresis, debouncing, or timeout logic.
     *
     * @warning **External Synchronization:** If SetFlag() is called from one thread while ShouldExit()
     *          is called from another, the results are well-defined by atomic semantics, but timing
     *          behavior may vary. Design your application accordingly.
     *
     * @see SetFlag()
     * @see Reset()
     * @see BaseStopCondition::ShouldExit()
     */
    bool ShouldExit(float error) override;

    /**
     * @brief Resets the internal flag to the "do not exit" state.
     * @details Clears the stop signal by setting the internal flag to false.
     *          After calling this method, ShouldExit() will return false until SetFlag(true) is called again.
     *          This method is typically called at the beginning of a new motion sequence.
     *
     * @post The internal flag is set to false.
     * @post ShouldExit() will return false until SetFlag(true) is called.
     *
     * @note **Mandatory:** Always call this method before starting a new motion sequence,
     *       even if the condition appears to be in the correct state. This ensures a clean,
     *       predictable starting point.
     *
     * @note **Thread-Safe:** This method uses atomic store semantics, allowing safe concurrent
     *       calls from multiple threads.
     *
     * @note **Idempotent:** Calling Reset() multiple times in succession has the same effect
     *       as calling it once.
     *
     * @note **No Side Effects:** This method only modifies the flag; it does not affect any
     *       other state or external systems.
     *
     * @warning **Timing Consideration:** If Reset() is called while another thread is in the middle
     *          of a motion loop, the flag will be cleared, potentially causing unexpected behavior.
     *          Coordinate timing to avoid race conditions.
     *
     * @see ShouldExit()
     * @see SetFlag()
     * @see BaseStopCondition::Reset()
     */
    void Reset() override;

    /**
     * @brief Sets the external stop signal to allow or prevent motion stopping.
     * @details Updates the internal flag, which directly controls whether ShouldExit() will return true.
     *          This is the primary method for external logic to control the stop condition.
     *          The operation is atomic and thread-safe.
     *
     * @param shouldExit true to signal that the robot should stop (ShouldExit() will return true);
     *                   false to signal that the robot should continue (ShouldExit() will return false).
     *
     * @post The internal flag is updated to the specified value.
     * @post Subsequent calls to ShouldExit() will return the updated value.
     *
     * @note **Immediate Effect:** The flag change takes effect immediately. If ShouldExit() is called
     *       after SetFlag(true), it will return true even if the error is zero or negative.
     *
     * @note **Thread-Safe:** This method safely updates the flag even when called concurrently
     *       from multiple threads. No explicit locking is required.
     *
     * @note **Use Cases:**
     *       - Emergency stop: `condition->SetFlag(true)` when safety system intervenes
     *       - User abort: `condition->SetFlag(true)` when user presses stop button
     *       - Conditional branching: `condition->SetFlag(some_external_condition)` based on sensor input
     *       - Composite conditions: Combine with other conditions using OrStopCondition or AndStopCondition
     *
     * @note **Clear Semantics:** The parameter name "shouldExit" makes the intent clear:
     *       true means "exit the motion loop", false means "continue the motion loop".
     *
     * @warning **No Automatic Reset:** Setting the flag to true does not automatically reset it.
     *          You must explicitly call SetFlag(false) or Reset() to clear the flag.
     *
     * @warning **No Debouncing:** The flag change takes effect immediately without debouncing.
     *          If your external signal is noisy (toggling rapidly), consider debouncing
     *          at the source.
     *
     * @see ShouldExit()
     * @see Reset()
     */
    void SetFlag(bool shouldExit);

protected:
    /**
     * @brief Constructs a new External Stop Condition.
     * @details Protected constructor; use the Create() factory method instead.
     *          Initializes the condition with the flag set to false.
     *
     * @post `_exitFlag` is initialized to false.
     * @post The condition is ready to evaluate stop signals.
     *
     * @note This constructor is called by the Create() factory method.
     */
    ExternalStopCondition();

    /**
     * @brief The atomic flag controlling the stop condition.
     * @details An atomic boolean that stores the external stop signal.
     *          Atomic semantics ensure thread-safe concurrent access from multiple threads.
     *
     * @invariant Always initialized to false by the constructor.
     * @invariant Modified by SetFlag() and Reset().
     * @invariant Read by ShouldExit().
     */
    std::atomic<bool> _exitFlag;
};

} // namespace Motion::Core::Robot
