/**
 * @file BaseProfileGenerator.h
 * @brief Provides the base interface for all motion profile generators.
 * @details Motion profile generators calculate smooth velocity (or other) profiles
 *          to achieve desired motion without exceeding acceleration constraints.
 */

#pragma once

#include <memory>

namespace Motion::Core::Robot {

#define ProfileGeneratorPointer(T) std::shared_ptr<T>

/**
 * @brief Defines the interface for motion profile generators.
 * @details This abstract class serves as the foundation for creating various velocity profiles
 *          over a specified distance. Concrete implementations will define the actual shape of the
 *          profile, such as constant velocity (step), trapezoidal, S-curve, or custom shapes.
 *
 *          A profile generator is responsible for smoothing motion commands to prevent jerky behavior
 *          and to respect acceleration/velocity constraints.
 *
 *          **Typical Usage:**
 *          1. Call `GenerateProfile(distance)` once to initialize the profile for a specific distance.
 *          2. Repeatedly call `CalculateValue(progress)` to retrieve the desired output (e.g., velocity)
 *             at various points of progress along the path.
 *          3. Repeat with a new distance when moving to a new target.
 *
 *          **Example:**
 *          ```cpp
 *          auto profile = TrapezoidalProfileGenerator::Create(1.0f, 2.0f); // 1.0 m/s², 2.0 m/s max
 *          profile->GenerateProfile(5.0f); // Generate profile for 5 meter move
 *          for (float progress = 0; progress <= 5.0f; progress += dt * velocity) {
 *              float velocity = profile->CalculateValue(progress);
 *              // Use velocity to command actuators
 *          }
 *          ```
 *
 * @note **Profile Characteristics:**
 *       - **Smooth:** Acceleration is continuous (no instantaneous jumps)
 *       - **Optimal:** Typically designed to reach the target in minimum time
 *       - **Safe:** Respects maximum acceleration and velocity constraints
 *
 * @see TrapezoidalProfileGenerator for trapezoidal profiles (common in robotics)
 * @see StepProfileGenerator for constant velocity profiles
 */
class BaseProfileGenerator {
public:
    /**
     * @brief Virtual destructor to ensure proper cleanup of derived classes.
     * @details Ensures that the destructor of the most-derived class is called when
     *          an object is deleted through a base class pointer. This is critical for
     *          polymorphic behavior.
     *
     * @note As a base class with virtual functions, the destructor must be virtual to allow for
     *       correct destruction of objects of derived classes through a base class pointer. This
     *       prevents resource leaks.
     */
    virtual ~BaseProfileGenerator() = default;

    /**
     * @brief Generates and prepares the motion profile for a given total distance.
     * @details This pure virtual function must be implemented by derived classes. It is responsible
     *          for calculating and caching the necessary parameters for the velocity profile based
     *          on the target distance. This method must be called before any calls to `CalculateValue()`.
     *
     *          During generation, the implementation should:
     *          1. Store the total distance for later reference
     *          2. Calculate critical points: acceleration distance, maximum reachable velocity, etc.
     *          3. Cache any parameters needed to quickly compute profile values later
     *          4. Validate the distance (> 0, not NaN, not Inf)
     *
     * @param distance The total distance the motion profile should cover.
     *                 Unit depends on the implementation context (typically meters or millimeters).
     *                 Must be a positive, finite non-zero value.
     *
     * @pre Before calling this method, the profile generator should be in a valid state
     *      (i.e., constructed and configured with valid parameters).
     *
     * @post After calling this method, `CalculateValue()` can be called for any progress value
     *       in the range [0, distance].
     *
     * @note **State Management:** This method resets the internal profile state.
     *       Calling it multiple times will discard the previous profile and generate a new one.
     *
     * @note **Profile Recalculation:** Each call to `GenerateProfile()` with a different distance
     *       recalculates the entire profile. For efficiency, reuse profiles when possible
     *       (i.e., moving the same distance repeatedly doesn't require regeneration).
     *
     * @note **Distance Interpretation:** The distance is typically along the primary motion axis
     *       (forward/backward for linear profiles, rotation angle for angular profiles).
     *
     * @note **Caching:** Implementations typically cache computed values to avoid expensive
     *       recalculation on every `CalculateValue()` call.
     *
     * @warning **Physical Realizability:** If the distance is too short given the maximum
     *          acceleration, the profile may not be achievable (e.g., insufficient space to reach
     *          maximum velocity). Derived classes should handle this gracefully
     *          (e.g., by reducing peak velocity).
     *
     * @warning **Invalid Input:** Passing distance <= 0, NaN, or Inf may cause undefined behavior
     *          or assertions in the derived implementation. Always validate input before calling.
     *
     * @see CalculateValue()
     */
    virtual void GenerateProfile(float distance) = 0;

    /**
     * @brief Calculates the profile's output value at a specific progress point.
     * @details Given the distance traveled so far (`progress`), this function returns the
     *          corresponding value from the generated profile (e.g., velocity, acceleration).
     *          This method is expected to be called repeatedly in a control loop as progress accumulates.
     *
     *          The method assumes `GenerateProfile()` has been called beforehand with a valid distance.
     *          Progress should advance from 0 to the total distance specified in `GenerateProfile()`.
     *
     * @param progress The distance traveled so far along the profile.
     *                 Should be in the range [0, total_distance] for meaningful results.
     *                 Unit must match the distance unit used in `GenerateProfile()`.
     *
     * @return float The calculated value from the profile at the given progress point.
     *               Typical return values:
     *               - **Velocity profile:** velocity in m/s (positive values)
     *               - **Custom profiles:** application-specific units
     *               The exact meaning depends on what the profile generator computes.
     *
     * @pre `GenerateProfile(distance)` must have been called before this method.
     *
     * @note **Interpolation:** Typically, this method interpolates between pre-calculated profile points
     *       using a lookup table and linear interpolation for efficiency.
     *
     * @note **Efficiency:** This method should be fast (microsecond-scale) because it is called in
     *       high-frequency control loops (100+ Hz). Avoid dynamic memory allocation, logging, or I/O.
     *
     * @note **Progress Ordering:** Typically, progress should increase monotonically (strictly increasing)
     *       for correct behavior. Non-monotonic or reverse-direction progress may cause undefined behavior.
     *
     * @note **Velocity Profile Interpretation:** For velocity profiles, a positive value means forward motion,
     *       and a negative value means backward motion.
     *
     * @note **Output Guarantees:** The returned value should honor the constraints specified when
     *       the profile generator was created (max velocity, max acceleration).
     *
     * @warning **Out-of-Range Progress:** The behavior for `progress` values outside [0, total_distance]
     *          is undefined and should be handled by the implementing class.
     *          - If progress > distance: may return constant velocity, zero, or undefined
     *          - If progress < 0: may return undefined or zero
     *          Derived classes should clearly document boundary behavior.
     *
     * @warning **Pre-Condition Violation:** If `GenerateProfile()` has not been called, or if called
     *          with invalid parameters, the return value is undefined. Always ensure proper initialization.
     *
     * @warning **NaN/Inf Propagation:** If mathematical operations result in NaN or Inf,
     *          these values might propagate through. Check derived implementations for proper handling.
     *
     * @see GenerateProfile()
     */
    virtual float CalculateValue(float progress) = 0;

protected:
    /**
     * @brief Default constructor for the BaseProfileGenerator.
     * @details Initializes the profile generator with default state.
     *          Derived classes may call this or provide their own initialization.
     */
    BaseProfileGenerator() {}
};

using BaseProfileGeneratorHandle = ProfileGeneratorPointer(BaseProfileGenerator);

} // namespace Motion::Core::Robot