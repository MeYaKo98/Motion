/**
 * @file AndStopCondition.h
 * @brief Defines a stop condition that aggregates multiple stop conditions using logical AND.
 * @details Implements the Composite design pattern for combining stop conditions.
 *          All conditions must be satisfied before the motion stops.
 */

#pragma once

#include "Motion/Core/Robot/StopCondition/BaseStopCondition.h"
#include <vector>

namespace Motion::Core::Robot {

/**
 * @brief A composite stop condition that is met only when **all** of its contained conditions are met.
 * @details This class implements the Composite design pattern for stop conditions, allowing
 *          treating a group of stop conditions as a single logical unit. The `ShouldExit` method
 *          returns true only if `ShouldExit` returns true for **all** contained conditions.
 *
 *          **Logical Definition:**
 *          ShouldExit(error) returns true if and only if:
 *          condition_A.ShouldExit(error) AND condition_B.ShouldExit(error)
 *
 *          **Practical Uses:**
 *          - "Stop when error is low AND velocity is low" (position and speed stability)
 *          - "Stop when position error < 1cm AND velocity error < 0.1 m/s"
 *          - "Stop when in tolerance window AND gyro has settled"
 *          - Multi-criteria safety checks where all must pass before declaring success
 *
 *          **Short-Circuit Evaluation:**
 *          The implementation uses logical AND semantics, which typically employ short-circuit
 *          evaluation. If condition_A returns false, condition_B may not be evaluated.
 *          However, both Reset() calls are typically made regardless for consistency.
 *
 * @note **Composability:** AndStopCondition instances can be nested to combine
 *       more than two conditions: (A && B) && (C && D)
 *
 * @note **Operator Overloading:** The `&&` operator is overloaded to create AndStopCondition
 *       instances conveniently: `condition_a && condition_b`
 *
 * @see BaseStopCondition for the base interface
 * @see OrStopCondition for combining with OR logic
 * @see ToleranceStopCondition for a simple tolerance-based condition
 */
class AndStopCondition : public BaseStopCondition{
public:
    /**
     * @brief Destroys the AndStopCondition object.
     * @details Cleans up the instance. The destructor does NOT delete the contained condition pointers.
     *
     * @note **No Deallocation of Children:** This destructor does not free the memory of the
     *       contained stop condition pointers. The creator of those condition objects is
     *       responsible for their cleanup. This is by design: conditions may be shared
     *       among multiple composites.
     */
    ~AndStopCondition();

    /**
     * @brief Determines if the robot should stop by evaluating both contained conditions.
     * @details Combines the results of both conditions using logical AND.
     *          Both conditions receive the same error value and must both return true
     *          for this composite to return true.
     *
     * @param error The current error value, passed unchanged to both contained conditions.
     *              The interpretation depends on what the conditions expect.
     *              Both conditions should document their error convention.
     *
     * @return true if **both** `_conditionsA.ShouldExit(error)` and `_conditionsB.ShouldExit(error)`
     *         return true. Returns false if either condition returns false.
     *
     * @note **Short-Circuit Evaluation:** While typical AND logic uses short-circuiting,
     *       implementations may evaluate both conditions regardless to ensure consistent state.
     *       Always assume both conditions' ShouldExit() method may be called.
     *
     * @note **Error Passing:** The error value is passed unchanged to both contained conditions.
     *       This assumes both conditions interpret the error in the same way (same units, same sign convention).
     *       If conditions have different error conventions, use separate error values or custom logic.
     *
     * @note **State Management:** Some contained conditions may have internal state.
     *       Ensure Reset() is called on the composite before the motion loop begins.
     *
     * @note **Performance:** This function evaluates all contained conditions, so its cost
     *       is the sum of the individual evaluation costs. Typically still O(1) for simple conditions.
     *
     * @warning **Error Convention Mismatch:** Be sure all conditions interpret the error
     *          uniformly. If one condition uses `|error|` and another uses `error` directly,
     *          results will be unexpected.
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
     * @post Both `_conditionsA` and `_conditionsB` have their state reset.
     *       They are ready to evaluate a new motion sequence.
     *
     * @note **Propagation:** Calling Reset() on a composite propagates the reset to all
     *       contained conditions, allowing nested composites to fully initialize.
     *
     * @note **Order:** Conditions are reset in order: A first, then B.
     *       For simple conditions with no side effects, order doesn't matter.
     *       Document any order dependencies in derived conditions.
     *
     * @warning **Before Every Motion:** Always call Reset() on the composite before
     *          starting a new motion loop, even if conditions are stateless.
     *          This ensures consistency and allows state-based conditions to initialize.
     *
     * @see _conditionsA
     * @see _conditionsB
     * @see BaseStopCondition::Reset()
     */
    void Reset() override;

    /**
     * @brief Friendship declaration for operator&& overload.
     * @details Allows the && operator to construct AndStopCondition instances with access
     *          to the protected constructor.
     *
     * @see operator&&(BaseStopConditionHandle, BaseStopConditionHandle)
     */
    friend BaseStopConditionHandle operator&&(BaseStopConditionHandle conditionA, BaseStopConditionHandle conditionB);

protected:
    /**
     * @brief Constructs a new AndStopCondition combining two conditions.
     * @details Protected constructor; typically called by the && operator overload.
     *          Creates a new composite containing the two conditions.
     *
     * @param conditionA The first condition in the AND operation.
     * @param conditionB The second condition in the AND operation.
     *
     * @post `_conditionsA` and `_conditionsB` are initialized with the provided handles.
     *
     * @note **Ownership:** The constructor does not take ownership of the condition pointers.
     *       The caller retains responsibility for lifetime management.
     *
     * @warning Ensure both conditions remain valid for the lifetime of this composite.
     */
    explicit AndStopCondition(BaseStopConditionHandle conditionA, BaseStopConditionHandle conditionB);

    /**
     * @brief Handle to the first condition (left operand of AND).
     * @details Evaluated first in the ShouldExit() method.
     *          May be another composite condition for nesting support.
     */
    BaseStopConditionHandle _conditionsA;

    /**
     * @brief Handle to the second condition (right operand of AND).
     * @details Evaluated second in the ShouldExit() method (unless short-circuited).
     *          May be another composite condition for nesting support.
     */
    BaseStopConditionHandle _conditionsB;
};

/**
 * @brief Operator overload for composing two stop conditions with AND logic.
 * @details Provides syntactic sugar for creating AndStopCondition instances.
 *          Allows natural expression of condition combinations.
 *
 * @param conditionA The first condition (left operand).
 * @param conditionB The second condition (right operand).
 *
 * @return BaseStopConditionHandle A handle to a new AndStopCondition instance.
 *
 * @note **Usage Example:**
 *       ```cpp
 *       auto tolerance = ToleranceStopCondition::Create(0.01f);
 *       auto timeout = TimeoutStopCondition::Create(5.0f); // 5 seconds (hypothetical)
 *       auto combined = tolerance && timeout; // Stop when BOTH conditions are met
 *       ```
 *
 * @warning **Ownership:** The created AndStopCondition owns the handles to the input conditions,
 *          but not the underlying condition objects. The original creator of the conditions
 *          retains cleanup responsibility.
 *
 * @see AndStopCondition
 */
BaseStopConditionHandle operator&&(BaseStopConditionHandle conditionA, BaseStopConditionHandle conditionB);

} // namespace Motion::Core::Robot