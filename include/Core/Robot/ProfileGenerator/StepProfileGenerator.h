/**
 * @file StepProfileGenerator.h
 * @brief Defines the StepProfileGenerator class for constant velocity motion profiles.
 * @details Provides a simple "step" profile that outputs constant velocity regardless of progress.
 */

#pragma once

#include "Core/Robot/ProfileGenerator/BaseProfileGenerator.h"
#include "Core/Logger.h"
#include <cmath>

namespace Motion::Core::Robot {

/**
 * @brief A concrete implementation of BaseProfileGenerator that produces a constant velocity profile.
 * @details This profile generator creates a "step" function that outputs a fixed commanded velocity
 *          regardless of the progress through the motion. The velocity jumps immediately to the
 *          configured speed and remains constant throughout the entire distance.
 *
 *          **Profile Characteristics:**
 *          - Velocity: constant throughout the motion
 *          - Acceleration: infinite (discontinuous jump at start)
 *          - Deceleration: infinite (discontinuous jump at end)
 *          - Time to target: distance / velocity (fixed, independent of progress)
 *
 *          **When to Use:**
 *          - Simple open-loop velocity control (no fancy profiling)
 *          - Situations where instant acceleration is acceptable
 *          - Starting point for basic motion before adding profiling
 *          - Testing/debugging: verifies motion works before adding complexity
 *
 *          **When NOT to Use:**
 *          - High-performance robotics requiring smooth motion
 *          - Precision applications sensitive to acceleration shocks
 *          - Low-power systems where energy efficiency matters
 *          - Situations with delicate cargo or mechanics
 *
 *          **Typical Use:**
 *          ```cpp
 *          auto profile = StepProfileGenerator::Create(1.0f); // 1.0 m/s constant
 *          profile->GenerateProfile(5.0f); // Move 5 meters
 *          float velocity = profile->CalculateValue(2.5f); // Returns 1.0 m/s at 2.5m progress
 *          ```
 *
 * @note **Simplicity:** This is the simplest profile generator. Use it when you don't need
 *       acceleration profiling or when testing control system basics.
 *
 * @note **Energy:** The instantaneous acceleration required means infinite jerk and potentially
 *       high mechanical stress. Real motors can't achieve this, so in practice there's an
 *       implicit acceleration phase determined by motor response.
 *
 * @compare Compare with TrapezoidalProfileGenerator for smooth acceleration-limited profiles.
 *
 * @see BaseProfileGenerator for the base interface
 * @see TrapezoidalProfileGenerator for more sophisticated profiles with acceleration limits
 */
class StepProfileGenerator;

using StepProfileGeneratorHandle = ProfileGeneratorPointer(StepProfileGenerator);

class StepProfileGenerator : public BaseProfileGenerator{
public:

    /**
     * @brief Factory method to create a StepProfileGenerator instance.
     * @details Creates a new step profile generator with the specified constant speed.
     *
     * @param speed The constant velocity to output throughout the motion.
     *              Positive values move forward; negative values move backward.
     *              Units must be consistent with the distance units used in GenerateProfile()
     *              (e.g., m/s if distances are in meters).
     *              Typical range: 0.1 to 100 (application-dependent).
     *
     * @return StepProfileGeneratorHandle A smart pointer to the new profile generator.
     *
     * @note **Zero Speed:** If speed = 0.0f, the robot will not move (velocity = 0).
     *       This is valid but unusual; typically used for debugging.
     *
     * @note **Negative Speed:** Negative speeds move the robot backward.
     *       The sign is preserved; CalculateValue() will return the negative speed.
     *
     * @note **Usage Pattern:**
     *       ```cpp
     *       auto forward = StepProfileGenerator::Create(1.0f); // 1.0 m/s forward
     *       auto backward = StepProfileGenerator::Create(-0.5f); // 0.5 m/s backward
     *       ```
     *
     * @warning There is no validation in this factory. Passing extremely large speeds
     *          is allowed but may result in infeasible motion when driving real hardware.
     */
    static StepProfileGeneratorHandle Create(float speed);

    /**
     * @brief Destroys the StepProfileGenerator object.
     * @details Performs any necessary cleanup.
     *          For this simple implementation, no special work is required.
     */
    ~StepProfileGenerator();

    /**
     * @brief Configures the profile for a specific travel distance.
     * @details Stores the target distance to define the motion bounds.
     *          Since the speed is pre-configured in the constructor, this method primarily
     *          validates and stores the distance. No complex calculation is needed—profiling
     *          is trivial (constant velocity).
     *
     * @param distance The total distance the motion profile should cover.
     *                 Positive values move forward; negative values move backward.
     *                 The distance is used to validate progress in CalculateValue().
     *                 Units must be consistent with the speed configured during creation.
     *
     * @post `_distance` is set to the provided value.
     *       Subsequent calls to CalculateValue() will be bounded by this distance.
     *
     * @post The profile is ready to be used. Call CalculateValue() to retrieve velocities.
     *
     * @note **Reusability:** Calling GenerateProfile() with different distances allows reusing
     *       the same generator for multiple motions of different lengths with the same speed.
     *
     * @note **Zero Distance:** Passing distance = 0.0f is valid but may cause the motion to
     *       exit immediately (progress >= distance). No motion will be executed.
     *
     * @note **Simplicity:** Unlike TrapezoidalProfileGenerator, no computation for acceleration
     *       ramps is needed. This makes the method very fast and lightweight.
     *
     * @note **Units:** The distance must be in the same units as the velocity.
     *       If velocity is m/s, distance should be in meters.
     *
     * @warning **Very Large Distances:** Passing extremely large distances is allowed
     *          but may cause numerical precision issues in integration loops.
     *          Keep distances in a reasonable range relative to the application.
     *
     * @see CalculateValue()
     */
    void GenerateProfile(float distance) override;

    /**
     * @brief Calculates the velocity at a given progress point.
     * @details Returns the constant speed configured in the constructor, regardless of
     *          the progress value. The progress parameter is essentially ignored (though
     *          some implementations may use it for bounds checking).
     *
     *          **Output Logic:**
     *          ```
     *          output_velocity = _speed (constant, regardless of progress)
     *          ```
     *
     * @param progress The distance traveled so far. Expected to be in the range [0, distance].
     *                 This parameter is ignored for velocity calculation but may be used
     *                 for validation or logging.
     *                 Units must match the distance units from GenerateProfile().
     *
     * @return float The configured constant speed value.
     *               - **Sign:** Preserved from construction (positive = forward, negative = backward)
     *               - **Magnitude:** Never changes, regardless of progress
     *               - **Range:** Usually [0.1, 100] but no validation is enforced
     *
     *               Examples:
     *               - If constructed with 2.0f, returns 2.0f at any progress
     *               - If constructed with -1.0f, returns -1.0f at any progress
     *
     * @note **Stationarity:** This function always returns the same value. There's no
     *       state change, no acceleration ramp, no deceleration. Completely predictable.
     *
     * @note **Efficiency:** O(1) operation—just returns a stored value.
     *       Extremely fast, suitable for high-frequency loops (1000+ Hz).
     *
     * @note **Progress Irrelevance:** The `progress` parameter is not used in the calculation.
     *       This makes the function particularly simple but also means no feedback-based
     *       adjustment is possible.
     *
     * @note **No Bounds Checking:** This implementation does not check if progress exceeds distance.
     *       Out-of-bounds progress returns the same speed. Bounds checking should be done
     *       by the caller (e.g., in the control loop via a stop condition).
     *
     * @warning **Open-Loop Nature:** Because velocity doesn't adapt to progress, the control
     *          loop must manage stopping (via a stop condition). Without a stop condition,
     *          the profile will output velocity indefinitely.
     *
     * @warning **Infinite Acceleration:** This profile represents instantaneous acceleration
     *          from rest to the configured speed. Real motors can't achieve this; there's an
     *          implicit ramp determined by motor dynamics. Account for this in tuning.
     *
     * @warning **No Deceleration:** There's no deceleration phase. The motor is expected to
     *          stop (command = 0) immediately when the stop condition is met. This may cause
     *          overshooting if momentum is significant.
     *
     * @see GenerateProfile()
     * @see BaseProfileGenerator::CalculateValue()
     */
    float CalculateValue(float progress) override;

protected:
    /**
     * @brief Constructs a StepProfileGenerator with a specific target speed.
     * @details Protected constructor; use the Create() factory method instead.
     *          Initializes the generator with a fixed velocity value.
     *
     * @param speed The constant velocity to be returned by the profile.
     *              Units are user-defined (e.g., m/s, rad/s).
     *              Positive for forward, negative for backward.
     *
     * @post `_speed` is set to the provided value.
     * @post The generator is ready for profile generation via GenerateProfile().
     *
     * @note This constructor is typically called by the Create() factory.
     */
    explicit StepProfileGenerator(float speed);

    /** @brief The constant speed output by this profile. Set in constructor, never changes. */
    float _speed;

    /** @brief The distance of the current profile. Set by GenerateProfile(). */
    float _distance;
};

} // namespace Motion::Core::Robot