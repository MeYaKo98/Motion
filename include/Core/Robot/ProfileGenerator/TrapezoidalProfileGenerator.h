/**
 * @file TrapezoidalProfileGenerator.h
 * @brief Defines the TrapezoidalProfileGenerator class for motion profile generation.
 * @details Implements smooth velocity profiles with constant acceleration and cruise phases.
 */

#pragma once
#include "BaseProfileGenerator.h"
#include "Core/Diagnostics/Logger.h"

namespace Motion::Core::Robot {

/**
 * @brief Generates a trapezoidal velocity profile for motion control.
 * @details This class calculates smooth velocity profiles that respect acceleration and
 *          maximum velocity constraints. The profile has up to three phases:
 *          1. **Acceleration Phase:** Velocity ramped up at constant acceleration from 0 to peak velocity
 *          2. **Cruise Phase:** Constant velocity at peak velocity (optional, may be skipped for short moves)
 *          3. **Deceleration Phase:** Velocity ramped down at constant acceleration from peak to 0
 *
 *          For shorter distances, the acceleration distance may be too large to reach the
 *          configured max velocity. In this case, a triangular profile is used instead:
 *          the velocity ramps up then immediately ramps down without a cruise phase.
 *
 *          **Profile Characteristics:**
 *          - Smooth acceleration (piecewise linear velocity, continuous jerk)
 *          - Optimal time-to-completion given acceleration constraints
 *          - Respects both acceleration and maximum velocity limits
 *          - No discontinuities in velocity (continuous motion)
 *
 *          **Mathematical Basis:**
 *          - Acceleration distance: d_accel = v_max² / (2 × a)
 *          - If 2 × d_accel > distance: Use triangular profile
 *          - Else: Use trapezoidal profile (with cruise phase)
 *          - Peak reachable velocity (triangular): v = √(2 × a × (distance / 2))
 *
 * @note **Typical Usage:** Used in motion control loops where smooth, bounded acceleration is desired.
 *       Examples: robot drives, elevator control, printer heads.
 *
 * @see BaseProfileGenerator for the base interface
 * @see StepProfileGenerator for constant velocity (no acceleration)
 */
class TrapezoidalProfileGenerator;

using TrapezoidalProfileGeneratorHandle = ProfileGeneratorPointer(TrapezoidalProfileGenerator);

class TrapezoidalProfileGenerator : public BaseProfileGenerator {
public:

    /**
     * @brief Factory method to create a TrapezoidalProfileGenerator instance.
     * @details Creates and validates a new trapezoidal profile generator with specified constraints.
     *
     * @param acceleration The rate of acceleration and deceleration magnitude (u/s²).
     *                     The absolute value is used internally, so sign is ignored.
     *                     Must be positive and non-zero.
     *                     Typical range: 0.1 to 50 (application-dependent units).
     *
     * @param velocity The maximum cruise velocity (u/s).
     *                 The absolute value is used internally, so sign is ignored.
     *                 Must be positive and non-zero.
     *                 Typical range: 0.1 to 100 (application-dependent units).
     *
     * @return TrapezoidalProfileGeneratorHandle A shared pointer to the new profile generator.
     *
     * @throws std::invalid_argument if acceleration == 0.0f
     * @throws std::invalid_argument if velocity == 0.0f
     *
     * @note **Units:** acceleration and velocity must have consistent units.
     *       If distance is in meters and time in seconds:
     *       - velocity in m/s (e.g., 1.0 for 1 m/s)
     *       - acceleration in m/s² (e.g., 2.0 for 2 m/s²)
     *
     * @note **Usage Pattern:**
     *       ```cpp
     *       auto profile = TrapezoidalProfileGenerator::Create(2.0f, 1.0f);
     *       // 2.0 m/s² acceleration, 1.0 m/s max velocity
     *       ```
     *
     * @warning Passing zero or negative acceleration/velocity will throw an exception.
     *          Always validate external inputs before passing to Create().
     */
    static TrapezoidalProfileGeneratorHandle Create(float acceleration, float velocity);

    /**
     * @brief Destroys the Trapezoidal Profile Generator.
     * @details Performs cleanup of any allocated resources.
     *          For this implementation, no special work is required.
     */
    ~TrapezoidalProfileGenerator() = default;

    /**
     * @brief Generates the motion profile for a specific distance.
     * @details Calculates the profile parameters (acceleration distance, peak velocity)
     *          based on the target distance and the configured acceleration/velocity limits.
     *          This method must be called before CalculateValue() is called.
     *
     *          **Algorithm:**
     *          1. Calculate acceleration distance: d_accel = v_max² / (2 × a)
     *          2. If 2 × d_accel > |distance|:
     *             - Triangular profile: reduce peak velocity to √(2 × a × |distance| / 2)
     *             - Acceleration phase reaches peak immediately, then decelerates
     *          3. Else:
     *             - Trapezoidal profile: maintain configured max velocity
     *             - Three phases: accelerate, cruise, decelerate
     *
     * @param distance The total distance to travel.
     *                 Positive distances move in the forward direction;
     *                 negative distances move backward.
     *                 The sign is tracked for profile generation.
     *                 Must be non-zero for a valid profile.
     *
     * @return void
     *
     * @post Internal state (`_distance`, `_accelerationDistance`, `_peakReachableVelocity`) is updated.
     *       Subsequent calls to CalculateValue() will generate values based on this profile.
     *
     * @note **Reusability:** Calling GenerateProfile() with a new distance discards the previous
     *       profile and generates a new one. Call this before each new motion.
     *
     * @note **Backward Motion:** Negative distances generate profiles that move backward.
     *       The velocity returned by CalculateValue() will be negative (opposite direction).
     *
     * @note **Distance Interpretation:** The distance must be in the same units as
     *       the velocity configured during creation. If velocity is m/s, distance should be in meters.
     *
     * @warning **Zero Distance:** Passing distance = 0.0f may cause issues.
     *          Some implementations may skip the motion; others may produce invalid profiles.
     *          Always ensure distance is non-zero.
     *
     * @warning **Very Small Distances:** For distances much smaller than the acceleration ramp,
     *          the resulting velocity may be very small (approaching 0). Integration in the control
     *          loop must handle this gracefully.
     *
     * @see CalculateValue()
     */
    void GenerateProfile(float distance) override;

    /**
     * @brief Calculates the target velocity at a specific progress point.
     * @details Given the distance traveled so far (`progress`), returns the velocity that should
     *          be used at that point in the profile. This is called repeatedly in the control loop
     *          as progress accumulates from 0 to the total distance.
     *
     *          **Velocity Calculation Phases:**
     *          1. **Acceleration Phase** (0 < progress < d_accel):
     *             v = √(2 × a × progress)
     *          2. **Cruise Phase** (d_accel ≤ progress ≤ distance - d_accel):
     *             v = v_max (constant)
     *          3. **Deceleration Phase** (distance - d_accel < progress < distance):
     *             v = √(2 × a × (distance - progress))
     *
     *          The sign of velocity depends on the direction of motion (positive/negative distance).
     *
     * @param progress The distance traveled so far along the profile.
     *                 Should be in the range [0, distance] for meaningful results.
     *                 Units must match the distance units used in GenerateProfile().
     *
     * @return float The calculated target velocity at the given progress point.
     *               - **Positive (forward):** for forward motion
     *               - **Negative (backward):** for backward motion
     *               - **Minimum magnitude:** Clamped to a minimum of 10.0f to prevent
     *                 zero velocity (which can cause numerical issues in control loops)
     *
     *               Examples:
     *               - Return +1.5 for forward motion at 1.5 m/s
     *               - Return -1.5 for backward motion at 1.5 m/s
     *               - Return ±10.0f (minimum) for very slow speeds
     *
     * @note **Minimum Velocity Clamp:** The output is clamped to a minimum magnitude of 10.0f.
     *       This prevents very slow speeds that can cause numerical instability or motor
     *       control dead-bands. Adjust this value if your application requires slower motion.
     *
     * @note **Continuity:** The output velocity is continuous (no jumps) even at phase boundaries.
     *       At the acceleration-cruise boundary, the transition is smooth.
     *
     * @note **Efficiency:** This function is called repeatedly in high-frequency loops (100-1000 Hz).
     *       The implementation uses only arithmetic operations and is O(1) fast.
     *
     * @note **Progress Interpretation:** Progress should advance monotonically from 0 to the total
     *       distance. The absolute progress value is used internally to handle both forward and
     *       backward motion.
     *
     * @warning **Out-of-Range Progress:** If progress is outside the range [0, distance]:
     *          - progress < 0: Returns 0 or undefine behavior (consult implementation)
     *          - progress > distance: Returns constant velocity, 0, or undefined (consult implementation)
     *          Always ensure progress is within the valid range in calling code.
     *
     * @warning **Pre-Condition:** GenerateProfile() must have been called before this method.
     *          If not, internal state is uninitialized and results are undefined.
     *
     * @warning **Minimum Velocity Impact:** The 10.0f minimum clamp may prevent the profile
     *          from reaching exactly zero velocity. If zero velocity is required at the end,
     *          handle stopping separately in the control loop.
     *
     * @note **Mathematical Stability:** Square root calculations (used for velocity ramping)
     *       may introduce small numerical errors. For high-precision applications,
     *       consider using precomputed lookup tables instead of on-the-fly calculations.
     *
     * @see GenerateProfile()
     * @see BaseProfileGenerator::CalculateValue()
     */
    float CalculateValue(float progress) override;

protected:
    /**
     * @brief Constructs a new Trapezoidal Profile Generator.
     * @details Protected constructor; use the Create() factory method instead.
     *          Initializes the generator with acceleration and velocity constraints.
     *
     * @param acceleration The rate of acceleration. The absolute value is used.
     * @param velocity The maximum cruise velocity. The absolute value is used.
     *
     * @post `_acceleration` and `_velocity` are initialized (using absolute values).
     * @post The generator is ready for profile generation via GenerateProfile().
     *
     * @note This constructor is typically called by the Create() factory after validation.
     */
    TrapezoidalProfileGenerator(float acceleration, float velocity);

    /** @brief Total distance of the current profile. Set by GenerateProfile(). */
    float _distance;

    /** @brief Configured acceleration rate (magnitude, always positive). */
    float _acceleration;

    /** @brief Configured maximum velocity (magnitude, always positive). */
    float _velocity;
    
    /** @brief Distance required to accelerate from 0 to peak velocity. Calculated by GenerateProfile(). */
    float _accelerationDistance;

    /** @brief Maximum velocity achievable for the current profile distance. May be less than _velocity if distance is short. */
    float _peakReachableVelocity;
};

} // namespace Motion::Core::Robot