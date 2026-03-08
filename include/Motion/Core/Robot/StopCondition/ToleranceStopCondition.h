/**
 * @file ToleranceStopCondition.h
 * @brief Defines a stop condition based on an error tolerance threshold.
 * @details This is the simplest and most common stop condition implementation.
 *          It stops when the error falls below a configured threshold.
 */

#pragma once

#include "Motion/Core/Robot/StopCondition/BaseStopCondition.h"
#include "Motion/Core/Logger.h"
#include <cmath>

namespace Motion::Core::Robot {

/**
 * @brief A stop condition that is satisfied when the error is within a specified tolerance.
 * @details This class implements a simple threshold-based stop criterion. It checks if the
 *          magnitude of the provided error is less than or equal to the configured tolerance value.
 *          It is commonly used in motion control to determine if a robot has reached its target
 *          within an acceptable margin of error.
 *
 *          **Mathematical Definition:**
 *          ShouldExit returns true if and only if: |error| <= tolerance
 *
 *          **Typical Uses:**
 *          - Position control: Stop when position error < 1cm
 *          - Velocity control: Stop when velocity error < 0.1 m/s
 *          - Angle control: Stop when angle error < 5 degrees
 *
 * @note **Tolerance Selection:**
 *       - Too tight (small tolerance): Robot overshoots, wastes energy, takes longer
 *       - Too loose (large tolerance): Robot stops prematurely, position inaccuracy
 *       - Typical values: 1-5% of target distance or 2-5 deg for rotation
 *
 * @warning **Sign Convention:** This condition uses absolute value (magnitude).
 *          Both positive and negative errors within the tolerance range will trigger the stop condition.
 *          No need to worry about error sign—only magnitude matters.
 *
 * @see BaseStopCondition for the base interface
 * @see AndStopCondition for combining conditions with AND logic
 * @see OrStopCondition for combining conditions with OR logic
 */
class ToleranceStopCondition; 

using ToleranceStopConditionHandle = StopConditionPointer(ToleranceStopCondition);

class ToleranceStopCondition : public BaseStopCondition{
public:

    /**
     * @brief Factory method to create a ToleranceStopCondition instance.
     * @details Creates and validates a new tolerance-based stop condition.
     *
     * @param tolerance The error threshold below which the stop condition is satisfied.
     *                  Must be a positive finite value. Units depend on the application context.
     *                  Examples: 0.01 (1cm), 0.05 (5 degrees), 0.001 (1mm)
     *
     * @return ToleranceStopConditionHandle A smart pointer to the newly created condition.
     *
     * @throws std::invalid_argument if tolerance <= 0
     * @throws std::invalid_argument if tolerance is NaN or Inf
     *
     * @note **Usage Pattern:**
     *       ```cpp
     *       auto stopCondition = ToleranceStopCondition::Create(0.01f); // ±1cm tolerance
     *       ```
     *
     * @warning Passing invalid tolerances will throw an exception.
     *          Always catch exceptions if tolerance comes from external sources.
     */
    static ToleranceStopConditionHandle Create(float tolerance);

    /**
     * @brief Destroys the Tolerance Stop Condition object.
     * @details Performs cleanup of any allocated resources.
     *          For this simple condition, no special cleanup is needed.
     */
    ~ToleranceStopCondition();

    /**
     * @brief Determines if the robot should stop based on the current error.
     * @details Evaluates whether the error magnitude is within the tolerance threshold.
     *          This is a simple, deterministic function with no internal state (stateless).
     *
     * @param error The current error value (e.g., target position - current position).
     *              The sign (positive/negative) is ignored; only magnitude matters.
     *              Units must match the tolerance threshold (same context).
     *
     * @return true if the absolute value of `error` is less than or equal to `_tolerance`;
     *         false otherwise.
     *         
     *         **Decision Logic:**
     *         ```
     *         if (fabs(error) <= _tolerance)
     *             return true;  // Error within tolerance—stop
     *         else
     *             return false; // Error exceeds tolerance—continue
     *         ```
     *
     * @note **Statelessness:** Unlike some stop conditions, this one is completely stateless.
     *       The decision depends only on the current error and the tolerance parameter.
     *       Calling Reset() has no effect (but is supported for API consistency).
     *
     * @note **No Hysteresis:** The condition uses a simple threshold with no hysteresis.
     *       If the error oscillates around the threshold, the condition may chatter
     *       (toggle on/off rapidly). For robustness, consider implementing hysteresis logic
     *       at the caller's level or using a windowing condition.
     *
     * @note **Absolute Value:** For both positive and negative errors, the condition
     *       behaves symmetrically. Error of +0.005 and -0.005 are treated identically
     *       if tolerance = 0.01.
     *
     * @note **Performance:** This function is O(1) and extremely fast (single comparison).
     *       Suitable for high-frequency control loops (1000+ Hz).
     *
     * @note **Thread-safety:** This function is const and touches only immutable state,
     *       making it inherently thread-safe (read-only).
     *
     * @warning **Sign Convention:** This condition ignores error sign.
     *          If error sign is significant to your application (e.g., asymmetric tolerance),
     *          implement custom logic in a derived class.
     *
     * @warning **Oscillation Risk:** At the boundary (error ≈ tolerance), small noise
     *          or quantization errors may cause the decision to oscillate.
     *          Use a larger tolerance or windowing condition if this is problematic.
     *
     * @warning **Tolerance == 0:** Passing tolerance = 0 in the factory will throw.
     *          Very small tolerances (< 1% of expected motion) may cause unreliable detection.
     *
     * @see Reset() for clearing state (no-op for this condition)
     * @see BaseStopCondition::ShouldExit()
     */
    bool ShouldExit(float error) override;

    /**
     * @brief Resets the internal state of the stop condition.
     * @details For this simple tolerance-based condition, there is no internal state to reset
     *          (the condition is stateless). This method is implemented for API consistency
     *          with BaseStopCondition.
     *
     * @note **No-Op:** This method does nothing for ToleranceStopCondition because the
     *       decision in ShouldExit() depends only on the tolerance and current error,
     *       not on any accumulated state.
     *
     * @note **API Consistency:** Even though this is a no-op, the method is provided to
     *       maintain compatibility with the BaseStopCondition interface. Code that uses
     *       stop conditions polymorphically should always call Reset().
     *
     * @note **Idempotency:** Calling Reset() multiple times has identical effect to calling
     *       it once (none).
     *
     * @warning **Composite Conditions:** If this condition is used as part of a composite
     *         condition (AndStopCondition, OrStopCondition), Reset() is called on the composite,
     *         which propagates to all contained conditions.
     *
     * @see BaseStopCondition::Reset()
     */
    void Reset() override;

protected:
    /**
     * @brief Constructs a new Tolerance Stop Condition.
     * @details Protected constructor; use the Create() factory method instead.
     *          Initializes the condition with the specified tolerance value.
     *
     * @param tolerance The threshold value for the stop condition.
     *                  The condition is met if the absolute error is less than or equal to this value.
     *                  Must be positive and finite.
     *
     * @post `_tolerance` is set to the provided value.
     * @post The condition is ready to evaluate errors.
     *
     * @note This constructor is called by the Create() factory after validation.
     */
    explicit ToleranceStopCondition(float tolerance);

    /**
     * @brief The configured error tolerance threshold.
     * @details Stored for comparison in the ShouldExit() method.
     *          Invariant throughout the lifetime of the condition.
     * @note This is a read-only member once set by the constructor.
     */
    float _tolerance;
};

} // namespace Motion::Core::Robot