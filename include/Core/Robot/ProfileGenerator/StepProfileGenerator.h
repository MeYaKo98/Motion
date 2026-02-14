/**
 * @file StepProfileGenerator.h
 * @brief Defines the StepProfileGenerator class for constant velocity motion profiles.
 * @details This file contains the definition of a profile generator that outputs a fixed velocity
 *          regardless of the progress, effectively simulating an infinite acceleration/deceleration impulse.
 */

#pragma once

#include "Core/Robot/ProfileGenerator/BaseProfileGenerator.h"
#include "Core/Diagnostics/Logger.h"
#include <cmath>

namespace Motion::Core::Robot {

/**
 * @brief A concrete implementation of BaseProfileGenerator that produces a constant velocity profile.
 * @details This profile generator creates a "step" function where the output velocity jumps immediately
 *          to the configured speed and remains constant throughout the entire distance.
 *          It does not account for acceleration or deceleration phases (effectively infinite acceleration).
 * @see BaseProfileGenerator
 */
class StepProfileGenerator : public BaseProfileGenerator{
public:
    /**
     * @brief Constructs a StepProfileGenerator with a specific target speed.
     * @details Initializes the generator with a fixed velocity value.
     * @param speed The constant speed to be returned by the profile. Units are user-defined (e.g., m/s).
     */
    explicit StepProfileGenerator(float speed);

    /**
     * @brief Destroys the StepProfileGenerator object.
     * @details Performs necessary cleanup.
     */
    ~StepProfileGenerator();

    /**
     * @brief Configures the profile for a specific travel distance.
     * @details Stores the target distance to define the bounds of the motion. 
     *          Since the speed is pre-configured in the constructor, this method primarily 
     *          validates the distance and prepares the generator for a new move.
     * @param distance The total distance the motion profile should cover.
     * @note This method must be called before the first call to CalculateValue() for a new movement.
     */
    void GenerateProfile(float distance) override;

    /**
     * @brief Calculates the velocity at a given progress point.
     * @details Returns the constant speed configured in the constructor, regardless of the progress value.
     * @param progress The distance traveled so far. Expected to be in the range [0, distance].
     * @return float The constant speed value.
     */
    float CalculateValue(float progress) override;

private:
    float _speed;
    float _distance;
};

} // namespace Motion::Core::Robot