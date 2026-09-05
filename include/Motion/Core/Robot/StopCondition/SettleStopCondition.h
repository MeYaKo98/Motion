/**
 * @file SettleStopCondition.h
 * @brief Defines a stop condition based on settling time after reaching tolerance.
 * @details This stop condition allows the robot to settle (remain within tolerance) for a
 *          specified duration before considering the motion complete. This is useful for
 *          ensuring stability and allowing oscillations to dampen.
 */

#pragma once

#include "Motion/Core/Robot/StopCondition/BaseStopCondition.h"
#include "Motion/Core/Logger.h"
#include <cmath>
#include <freertos/FreeRTOS.h>

namespace Motion::Core::Robot {

/**
 * @brief A stop condition that is satisfied when the error remains within tolerance for a specified time.
 * @details This class implements a time-based stop criterion combined with a tolerance threshold.
 *          The condition checks if the error is within tolerance AND if it has remained there for
 *          a minimum duration (configurable settle time).
 *
 *          **State Transitions:**
 *          1. Initially: Waiting for error to enter tolerance range
 *          2. Once error enters tolerance: Start settle timer
 *          3. If error exits tolerance: Reset timer and go back to state 1
 *          4. If error stays in tolerance for targetSettleTime: Return true (exit)
 *
 *          **Mathematical Definition:**
 *          - Phase 1 (entering tolerance): When |error| <= tolerance and timer not started
 *          - Phase 2 (settling): When timer elapsed >= targetSettleTime and |error| still <= tolerance
 *          - Returns true only after Phase 2 is complete
 *
 *          **Typical Uses:**
 *          - Ensure an articulated arm settles before releasing a held object
 *          - Verify a wheeled robot comes to complete rest after reaching position
 *          - Allow pendulum oscillations to dampen after reaching target
 *          - Provide stability confirmation in high-precision positioning tasks
 *
 * @note **Settle Time Selection:**
 *       - Too short: Motion may not be fully stabilized, continuing oscillations
 *       - Too long: Unnecessary delay before motion completes
 *       - Typical values: 500ms - 2000ms depending on system inertia
 *
 * @note **Timer Basis:**
 *       - Uses FreeRTOS tick count for timing
 *       - Measurement is in milliseconds after conversion via pdTICKS_TO_MS()
 *       - Immune to system load variations (tick-based, not wall-clock)
 *
 * @warning **Stateful:** This condition maintains internal state (settle start time).
 *          Must call Reset() before each motion sequence.
 *
 * @warning **Tolerance Dependency:** This condition requires error to be within tolerance.
 *          If tolerance is too tight, the robot may never settle. Select appropriate
 *          tolerance based on system dynamics.
 *
 * @see BaseStopCondition for the base interface
 * @see ToleranceStopCondition for simple tolerance-based stopping
 * @see OscillationStopCondition for oscillation-count-based stopping
 */
class SettleStopCondition;

using SettleStopConditionHandle = StopConditionPointer(SettleStopCondition);

class SettleStopCondition : public BaseStopCondition {
public:

    /**
     * @brief Factory method to create a SettleStopCondition instance.
     * @details Creates and validates a new settle-time-based stop condition.
     *
     * @param tolerance The error threshold that must be maintained for settle time.
     *                  Must be positive and finite. When |error| > tolerance, the
     *                  settle timer is reset.
     *                  Units must match the error metric used in ShouldExit().
     *                  Examples: 0.01 (1cm), 0.05 (5 degrees)
     *
     * @param targetSettleTime The minimum time (in milliseconds) that the error must remain
     *                         within tolerance before the condition is satisfied.
     *                         Must be positive. Typical values: 500-2000ms.
     *
     * @return SettleStopConditionHandle A smart pointer to the newly created condition.
     *
     * @throws std::invalid_argument if tolerance <= 0
     * @throws std::invalid_argument if targetSettleTime <= 0
     *
     * @note **Usage Pattern:**
     *       ```cpp
     *       auto stopCondition = SettleStopCondition::Create(
     *           0.01f,    // 1cm tolerance
     *           1000      // 1 second settle time (in ms)
     *       );
     *       ```
     *
     * @see Reset()
     * @see ShouldExit()
     */
    static SettleStopConditionHandle Create(float tolerance, uint32_t targetSettleTime);

    /**
     * @brief Destroys the Settle Stop Condition object.
     * @details Performs cleanup of any allocated resources.
     */
    ~SettleStopCondition();

    /**
     * @brief Determines if the robot should stop based on settled state within tolerance.
     * @details Evaluates whether the error has remained within the tolerance range for the
     *          required settle duration. The process:
     *          1. If |error| > tolerance: Reset timer, return false
     *          2. If |error| <= tolerance:
     *             - If first time in tolerance: Initialize settle timer
     *             - If elapsed time since settling started >= targetSettleTime: return true
     *             - Otherwise: return false
     *
     * @param error The current error value (e.g., target position - current position).
     *              The sign is ignored; only magnitude matters for tolerance checking.
     *              Units must match the tolerance threshold.
     *
     * @return true if error has remained within tolerance for at least targetSettleTime;
     *         false if error exceeds tolerance or settle time has not elapsed.
     *
     * @post If this method returns true, the robot is considered to have settled.
     *
     * @pre The condition should have been reset via Reset() before the motion loop begins.
     *
     * @note **Frequency:** This method is typically called in a control loop at high frequency.
     *       Internally tracks only tick counts for minimal overhead.
     *
     * @note **Stateful:** This method maintains internal state (_settleStartTime).
     *       The same error at different times may produce different results.
     *       Always call Reset() before starting a new motion sequence.
     *
     * @note **Time Measurement:** Uses FreeRTOS tick counts. Works correctly even if
     *       the tick count wraps around (uint32_t overflow handling).
     *
     * @warning **Reset Required:** If ShouldExit() has been called before, you must call
     *          Reset() to clear the internal timer state before starting a new motion sequence.
     *
     * @see Reset() for clearing internal state between motion sequences
     * @see BaseStopCondition::ShouldExit()
     */
    bool ShouldExit(float error) override;

    /**
     * @brief Resets the internal state of the stop condition.
     * @details Clears the settle timer, allowing a fresh measurement during the next motion sequence.
     *          After calling this, the condition will wait for the error to enter tolerance
     *          and remain there for the full settle duration.
     *
     * @post _settleStartTime is cleared (set to an initialized but invalid state)
     * @post The next call to ShouldExit() will restart the settle timer when error enters tolerance
     * @post Idempotent: calling Reset() multiple times is safe
     *
     * @note **Mandatory:** Always call this method at the beginning of each motion sequence.
     *
     * @note **Composite Conditions:** If this condition is used within a composite
     *       (AndStopCondition, OrStopCondition), Reset() is called on the composite,
     *       which propagates to all contained conditions.
     *
     * @see ShouldExit()
     * @see BaseStopCondition::Reset()
     */
    void Reset() override;

protected:
    /**
     * @brief Constructs a new Settle Stop Condition.
     * @details Protected constructor; use the Create() factory method instead.
     *
     * @param tolerance The error tolerance threshold.
     * @param targetSettleTime The required settle duration in milliseconds.
     *
     * @post _tolerance is set to |tolerance|
     * @post _targetSettleTime is set to targetSettleTime
     * @post _settleStartTime is initialized to 0 (not settling yet)
     */
    SettleStopCondition(float tolerance, uint32_t targetSettleTime);

    /**
     * @brief The configured error tolerance threshold.
     * @details Error must remain below this value for the settle timer to advance.
     *          Invariant throughout the lifetime of the condition.
     */
    float _tolerance;

    /**
     * @brief The minimum duration (in milliseconds) error must remain within tolerance.
     * @details Once error enters tolerance, this timer begins. If it completes without
     *          the error exiting tolerance, the condition is satisfied.
     *          Invariant throughout the lifetime of the condition.
     */
    uint32_t _targetSettleTime;

    /**
     * @brief The FreeRTOS tick count when the error entered tolerance.
     * @details Used to measure elapsed settle time. Reset to 0 when error exits tolerance
     *          or when Reset() is called.
     * @invariant Either 0 (not settling) or a valid tick count (currently settling)
     */
    TickType_t _settleStartTime;

    /**
     * @brief Flag indicating if we are currently in the settling phase.
     * @details True if error is within tolerance and we're measuring settle time;
     *          False if error is outside tolerance or settling has not started.
     */
    bool _isSettling;
};

} // namespace Motion::Core::Robot
