/**
 * @file ORStopCondition.h
 * @brief Defines a stop condition that aggregates multiple stop conditions using logical OR.
 * @details Implements the Composite design pattern for combining stop conditions.
 *          If any condition is satisfied, the motion stops.
 */

#pragma once

#include "Core/Robot/StopCondition/BaseStopCondition.h"
#include <vector>

namespace Motion::Core::Robot {

/**
 * @brief A composite stop condition that is met when **any** of its contained conditions are met.
 * @details This class implements the Composite design pattern for stop conditions, allowing
 *          treating a group of stop conditions as a single logical unit. The `ShouldExit` method
 *          returns true if `ShouldExit` returns true for **any** of the contained conditions.
 *
 *          **Logical Definition:**
 *          ShouldExit(error) returns true if and only if:
 *          condition_A.ShouldExit(error) OR condition_B.ShouldExit(error)
 *
 *          **Practical Uses:**
 *          - Safety checks: "Stop if target reached OR obstacle detected"
 *          - Timeout behavior: "Stop if in tolerance OR timeout exceeded"
 *          - Fallback logic: "Stop if close enough OR give up after time"
 *          - Multi-sensor logic: "Stop if encoder position OK OR IMU detects collision"
 *
 *          **Short-Circuit Evaluation:**
 *          The implementation typically uses short-circuit evaluation. If condition_A
 *          returns true, the entire result is true and condition_B may not be evaluated.
 *          However, both Reset() calls are typically made for consistency.
 *
 * @note **Composability:** ORStopCondition instances can be nested for complex logic:
 *       (A || B) || (C || D)
 *
 * @note **Operator Overloading:** The `||` operator is overloaded for convenient condition
 *       combination: `condition_a || condition_b`
 *
 * @note **Safety Application:** Common for safety-critical systems where any single
 *       failure criterion should trigger an immediate stop (fail-safe behavior).
 *
 * @see BaseStopCondition for the base interface
 * @see AndStopCondition for combining with AND logic
 * @see ToleranceStopCondition for a simple tolerance-based condition
 */
class ORStopCondition : public BaseStopCondition{
public:
    /**
     * @brief Destroys the ORStopCondition object.
     * @details Cleans up the composite instance. The destructor does NOT delete the contained
     *          condition pointer objects.
     *
     * @note **No Deallocation of Children:** This destructor does not free the memory of the
     *       contained stop condition pointers held in `_conditionsA` and `_conditionsB`.
     *       The creator of those condition objects is responsible for their cleanup.
     *       This is by design: conditions may be shared among multiple composite instances.
     *
     * @note **Ownership Model:** The ORStopCondition holds handles (references) to conditions,
     *       not ownership. Multiple composites can reference the same condition object.
     *
     * @warning **Memory Safety:** If you create conditions and pass them to this composite,
     *          you are responsible for deleting them when done. Use smart pointers
     *          (already provided by BaseStopConditionHandle's shared_ptr) for automatic cleanup.
     */
    ~ORStopCondition();

    /**
     * @brief Determines if the robot should stop by evaluating the contained conditions.
     * @details Combines the results of both conditions using logical OR.
     *          If either condition returns true, this composite returns true immediately
     *          (short-circuit evaluation typically used).
     *
     * @param error The current error value, passed unchanged to both contained conditions.
     *              The interpretation depends on what the conditions expect.
     *              Both conditions should document their error convention.
     *
     * @return true if **any** contained condition is satisfied:
     *         `_conditionsA.ShouldExit(error) || _conditionsB.ShouldExit(error)`
     *         Returns false only if both conditions return false.
     *
     * @note **Short-Circuit Evaluation:** While typical OR logic uses short-circuiting,
     *       implementations may evaluate both conditions regardless to ensure consistent state.
     *       Always assume both conditions' ShouldExit() method may be called.
     *
     * @note **Error Passing:** The error value is passed unchanged to both conditions.
     *       Both should interpret the error in the same way (units, sign convention).
     *       If different error conventions are needed, use separate error values or custom logic.
     *
     * @note **State Management:** Some conditions may maintain internal state.
     *       Ensure Reset() is called on the composite before the motion loop begins.
     *
     * @note **Performance:** Evaluates all contained conditions, so cost is the sum of
     *       individual evaluation costs. Typically still O(1) for simple conditions.
     *
     * @warning **Error Convention Mismatch:** Ensure all conditions interpret the error
     *          uniformly. Inconsistency leads to unpredictable stop behavior.
     *
     * @see _conditionsA
     * @see _conditionsB
     * @see BaseStopCondition::ShouldExit()
     */
    bool ShouldExit(float error) override;

    /**
     * @brief Resets the internal state of all contained stop conditions.
     * @details Iterates through both contained conditions and calls `Reset()` on each,
     *          ensuring they are ready for a new motion sequence.
     *
     * @return void
     *
     * @post Both `_conditionsA` and `_conditionsB` have their internal state reset.
     *       They are ready to evaluate a new motion sequence.
     *
     * @note **Propagation:** Calling Reset() on a composite propagates to all contained
     *       conditions, enabling proper initialization even of nested composites.
     *
     * @note **Order:** Conditions are reset in order: A first, then B.
     *       For simple conditions with no side effects, order is irrelevant.
     *       Document any order-dependent behavior in custom conditions.
     *
     * @warning **Before Every Motion:** Always call Reset() on the composite before
     *          starting a new motion loop. This ensures all conditions (especially
     *          state-based ones) start fresh.
     *
     * @see _conditionsA
     * @see _conditionsB
     * @see BaseStopCondition::Reset()
     */
    void Reset() override;

    /**
     * @brief Friendship declaration for operator|| overload.
     * @details Allows the || operator to construct ORStopCondition instances with access
     *          to the protected constructor.
     *
     * @see operator||(BaseStopConditionHandle, BaseStopConditionHandle)
     */
    friend BaseStopConditionHandle operator||(BaseStopConditionHandle conditionA, BaseStopConditionHandle conditionB);

protected:
    /**
     * @brief Constructs a new ORStopCondition combining two conditions.
     * @details Protected constructor; typically called by the || operator overload.
     *          Creates a new composite instance containing the two conditions.
     *
     * @param conditionA The first condition in the OR operation ( left operand).
     * @param conditionB The second condition in the OR operation (right operand).
     *
     * @post `_conditionsA` and `_conditionsB` are initialized with the provided handles.
     *
     * @note **Ownership:** The constructor does not take ownership of condition pointers.
     *       The caller retains responsibility for lifetime management.
     *
     * @warning Ensure both conditions remain valid for the lifetime of this composite.
     */
    explicit ORStopCondition(BaseStopConditionHandle conditionA, BaseStopConditionHandle conditionB);

    /**
     * @brief Handle to the first condition (left operand of OR).
     * @details Evaluated first in the ShouldExit() method.
     *          May be another composite condition for nesting support.
     */
    BaseStopConditionHandle _conditionsA;

    /**
     * @brief Handle to the second condition (right operand of OR).
     * @details Evaluated second in the ShouldExit() method (unless short-circuited).
     *          May be another composite condition for nesting support.
     */
    BaseStopConditionHandle _conditionsB;
};

/**
 * @brief Operator overload for composing two stop conditions with OR logic.
 * @details Provides syntactic sugar for creating ORStopCondition instances.
 *          Allows natural, readable condition composition.
 *
 * @param conditionA The first condition (left operand).
 * @param conditionB The second condition (right operand).
 *
 * @return BaseStopConditionHandle A handle to a new ORStopCondition instance.
 *
 * @note **Usage Example:**
 *       ```cpp
 *       auto tolerance = ToleranceStopCondition::Create(0.01f);
 *       auto timeout = TimeoutStopCondition::Create(5.0f); // 5 seconds (hypothetical)
 *       auto safeStop = tolerance || timeout; // Stop when EITHER condition is met
 *       ```
 *
 * @note **Safety Application:** Useful for failsafe: stop on success OR on timeout/error.
 *
 * @warning **Ownership:** The created ORStopCondition owns handles to the input conditions,
 *          but not the underlying objects. Original creators retain cleanup responsibility.
 *
 * @see ORStopCondition
 */
BaseStopConditionHandle operator||(BaseStopConditionHandle conditionA, BaseStopConditionHandle conditionB);

} // namespace Motion::Core::Robot