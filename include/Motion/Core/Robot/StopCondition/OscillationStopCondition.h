/**
 * @file OscillationStopCondition.h
 * @brief Defines a stop condition based on counting error oscillations about zero.
 * @details This stop condition allows the robot to stop after the error has oscillated
 *          (crossed zero) a specified number of times while within tolerance. This is useful
 *          for damping-based systems where oscillations gradually settle.
 */

#pragma once

#include "Motion/Core/Robot/StopCondition/BaseStopCondition.h"
#include "Motion/Core/Logger.h"
#include <cmath>

namespace Motion::Core::Robot {

/**
 * @brief A stop condition that is satisfied when error oscillations reach a target count.
 * @details This class implements an oscillation-counting stop criterion combined with a tolerance threshold.
 *          The condition counts how many times the error crosses zero (changes sign) while
 *          remaining within tolerance, and stops once a target oscillation count is reached.
 *
 *          **Oscillation Detection:**
 *          An oscillation is detected when the sign of the error changes from one call to the next:
 *          - Phase transition: currentError > 0 && previousError < 0 (or vice versa)
 *          - Only counts if both currentError and previousError are within tolerance range
 *
 *          **State Transitions:**
 *          1. Initially: monitoring for sign changes of error
 *          2. When sign change detected AND both values in tolerance AND inTolerance: increment counter
 *          3. When oscillation count reaches target: return true (exit)
 *          4. If error outside tolerance AND oscillation count >= target AND inTolerance: return true
 *
 *          **Typical Uses:**
 *          - Stop a servo after damped oscillations settle (N oscillations = stable)
 *          - Control a resonant system by letting it ring down N times before stopping
 *          - Implement custom settling logic based on motion resonance patterns
 *          - Combine with tolerance check for robust damping detection
 *
 * @note **Oscillation Count Selection:**
 *       - Count = 1: Stop after first sign change (fast but may exit prematurely)
 *       - Count = 2-3: Typical for many servos (error overshoots once or twice)
 *       - Count = 4+: For systems with significant inertia or compliance
 *
 * @note **Tolerance Dependency:**
 *       - Error must be within tolerance to count as valid oscillation
 *       - Acts as both a stopping criterion AND a validation gate for oscillation counting
 *       - Choose tolerance to capture typical damped oscillations
 *
 * @note **State Tracking:**
 *       - Maintains previous error value to detect sign crossings
 *       - Maintains oscillation counter across multiple calls
 *       - Use Reset() to clear counters between motion sequences
 *
 * @warning **Stateful:** This condition maintains internal state (oscillation count, previous error).
 *          Must call Reset() before each motion sequence.
 *
 * @warning **Edge Case - Zero Error:** If error is exactly zero (or very close due to floating point),
 *          it may not trigger sign-change detection. The first call with zero might not increment
 *          the oscillation counter. Consider error quantization and filtering in caller.
 *
 * @warning **Sign Convention:** Both positive and negative errors are treated symmetrically.
 *          A crossing from -0.01 to +0.01 (in tolerance) counts as one oscillation.
 *
 * @see BaseStopCondition for the base interface
 * @see ToleranceStopCondition for simple tolerance-based stopping
 * @see SettleStopCondition for time-based settling detection
 */
class OscillationStopCondition;

using OscillationStopConditionHandle = StopConditionPointer(OscillationStopCondition);

class OscillationStopCondition : public BaseStopCondition {
public:

    /**
     * @brief Factory method to create an OscillationStopCondition instance.
     * @details Creates and validates a new oscillation-counting stop condition.
     *
     * @param targetOscillations The number of complete oscillations (sign changes) required
     *                          before the stop condition is satisfied.
     *                          Must be positive (typically 1-4).
     *                          Examples:
     *                          - 1: Stop after first overshoot corrects (one sign change)
     *                          - 2: Allow one complete oscillation cycle
     *                          - 3: Allow system to settle through multiple overshoots
     *
     * @return OscillationStopConditionHandle A smart pointer to the newly created condition.
     *
     * @throws std::invalid_argument if targetOscillations <= 0
     *
     * @note **Usage Pattern:**
     *       ```cpp
     *       auto stopCondition = OscillationStopCondition::Create(
     *           2        // Stop after 2 oscillations
     *       );
     *       ```
     *
     * @see Reset()
     * @see ShouldExit()
     */
    static OscillationStopConditionHandle Create(uint32_t targetOscillations);

    /**
     * @brief Destroys the Oscillation Stop Condition object.
     * @details Performs cleanup of any allocated resources.
     */
    ~OscillationStopCondition();

    /**
     * @brief Determines if the robot should stop based on oscillation count.
     * @details Evaluates whether the error has oscillated (crossed zero) the required number
     *          of times while remaining within tolerance. The process:
     *          1. Detect oscillation: sign change between currentError and previousError
     *          2. If oscillation detected : increment currentOscillationCount
     *          3. Return true if: (currentOscillationCount >= targetOscillations)
     *          5. Update previousError for next call
     *
     * @param error The current error value. Typically: target - current (signed).
     *              Both positive and negative values are treated symmetrically.
     *              Units must match the tolerance threshold.
     *              Sign changes from call to call are used to detect oscillations.
     *
     * @return true if the oscillation count has reached the target;
     *         false otherwise.
     *
     * @post The previous error is updated for the next call (for sign-change detection).
     * @post If an oscillation is detected, currentOscillationCount is incremented.
     *
     * @pre The condition should have been reset via Reset() before the motion loop begins.
     *
     * @note **Frequency:** This method is typically called in a control loop at high frequency (100+ Hz).
     *       Internally maintains only integer counters and floating-point values for minimal overhead.
     *
     * @note **Stateful:** This method maintains internal state (_previousError, _currentOscillationCount).
     *       The same error at different times may produce different results depending on state.
     *       Always call Reset() before starting a new motion sequence.
     *
     * @note **Sign Detection:** Uses comparison operators to detect sign changes:
     *       - `currentError > 0 && previousError < 0` detects crossing from negative to positive
     *       - `currentError < 0 && previousError > 0` detects crossing from positive to negative
     *       - Zero values are excluded from sign-change detection
     *
     * @warning **Reset Required:** You must call Reset() to clear internal counters before
     *          starting a new motion sequence. Otherwise, oscillation counts from previous
     *          motions will carry over.
     *
     * @warning **First Call:** The first call to ShouldExit() always sees previousError = 0.
     *          This may trigger a sign-change detection if the initial error has a different sign.
     *          Consider this when analyzing the first oscillation count.
     *
     * @warning **Floating Point Comparison:** Error values near zero may have quantization or
     *          noise leading to spurious sign changes. If this is problematic, apply filtering
     *          or use a deadzone in the caller.
     *
     * @see Reset() for clearing oscillation count between motion sequences
     * @see BaseStopCondition::ShouldExit()
     */
    bool ShouldExit(float error) override;

    /**
     * @brief Resets the internal state of the stop condition.
     * @details Clears the oscillation counter and previous error tracking, allowing a fresh
     *          measurement during the next motion sequence. After calling this, the condition
     *          will begin counting oscillations from zero.
     *
     * @post _currentOscillationCount is set to 0
     * @post The next call to ShouldExit() will start fresh oscillation counting
     * @post Idempotent: calling Reset() multiple times is safe
     *
     * @note **Mandatory:** Always call this method at the beginning of each motion sequence.
     *       Otherwise, oscillation counts from previous motions will accumulate.
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
     * @brief Constructs a new Oscillation Stop Condition.
     * @details Protected constructor; use the Create() factory method instead.
     *
     * @param targetOscillations The target number of oscillations to count.
     *
     * @post _targetOscillations is set to targetOscillations
     * @post _currentOscillationCount is set to 0
     * @post _previousError is set to 0
     */
    OscillationStopCondition(uint32_t targetOscillations);

    /**
     * @brief The target number of oscillations required to satisfy the stop condition.
     * @details When _currentOscillationCount reaches or exceeds this value (and error
     *          is within tolerance), ShouldExit() returns true.
     *          Invariant throughout the lifetime of the condition.
     */
    uint32_t _targetOscillations;

    /**
     * @brief The count of oscillations detected so far.
     * @details Incremented each time a sign change is detected in the error signal.
     *          Reset to 0 by Reset().
     *
     * @invariant _currentOscillationCount >= 0
     * @invariant _currentOscillationCount is only incremented, never decremented
     */
    uint32_t _currentOscillationCount;

    /**
     * @brief The error value from the previous ShouldExit() call.
     * @details Used to detect sign changes (oscillations) in the error signal.
     *          Initialized to 0 by Reset(), then updated each call to ShouldExit().
     *
     * @invariant Updated at the end of every ShouldExit() call
     * @invariant Used for sign-change detection only; its magnitude is not compared
     */
    float _previousError;
};

} // namespace Motion::Core::Robot
