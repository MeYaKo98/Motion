/**
 * @file BaseProfileGenerator.h
 * @brief Provides the base interface for all motion profile generators.
 */

#pragma once

namespace Motion::Core::Robot {

/**
 * @brief Defines the interface for motion profile generators.
 * @details This abstract class serves as the foundation for creating various velocity profiles
 *          over a specified distance. Concrete implementations will define the actual shape of the
 *          profile, such as constant velocity, trapezoidal, or S-curve.
 *
 *          The typical usage involves two steps:
 *          1. Call GenerateProfile() once to configure the profile for a specific total distance.
 *          2. Repeatedly call CalculateValue() to retrieve the desired output (e.g., velocity)
 *             at various points of progress along the path.
 */
class BaseProfileGenerator {
public:
    /**
     * @brief Default constructor for the BaseProfileGenerator.
     */
    BaseProfileGenerator() {}

    /**
     * @brief Virtual destructor to ensure proper cleanup of derived classes.
     * @note As a base class with virtual functions, the destructor must be virtual to allow for
     *       correct destruction of objects of derived classes through a base class pointer. This
     *       prevents resource leaks.
     */
    virtual ~BaseProfileGenerator() = default;

    /**
     * @brief Generates and prepares the motion profile for a given total distance.
     * @details This pure virtual function must be implemented by derived classes. It is responsible
     *          for calculating and caching the necessary parameters for the velocity profile based
     *          on the target distance. This method should be called before any calls to CalculateValue().
     * @param distance The total distance the motion profile should cover.
     * @note This method should reset any previous profile data and generate a new one.
     */
    virtual void GenerateProfile(float distance) = 0;

    /**
     * @brief Calculates the profile's output value at a specific progress point.
     * @details Given the distance traveled so far (`progress`), this function returns the
     *          corresponding value from the generated profile (e.g., velocity). It is expected
     *          that GenerateProfile() has been called before using this method.
     * @param progress The distance traveled so far along the profile, from 0 to the total distance.
     * @return float The calculated value from the profile at the given progress point (e.g., velocity).
     * @warning The behavior for `progress` values outside the range [0, total_distance] is
     *          undefined and should be handled by the implementing class.
     */
    virtual float CalculateValue(float progress) = 0;
};

} // namespace Motion::Core::Robot