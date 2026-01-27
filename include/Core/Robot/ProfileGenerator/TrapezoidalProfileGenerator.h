/**
 * @file TrapezoidalProfileGenerator.h
 * @brief Defines the TrapezoidalProfileGenerator class for motion profile generation.
 */

#pragma once
#include "BaseProfileGenerator.h"

namespace Motion::Core::Robot {

/**
 * @brief Generates a trapezoidal velocity profile for motion control.
 * 
 * This class calculates the velocity required to traverse a path with a specified
 * acceleration and maximum velocity. It handles both trapezoidal (acceleration,
 * cruise, deceleration) and triangular (acceleration, deceleration) profiles
 * depending on the distance.
 */
class TrapezoidalProfileGenerator : public BaseProfileGenerator {
public:
    /**
     * @brief Constructs a new Trapezoidal Profile Generator.
     * 
     * @param acceleration The rate of acceleration and deceleration. The absolute value is used.
     * @param velocity The maximum cruise velocity. The absolute value is used.
     */
    TrapezoidalProfileGenerator(float acceleration, float velocity);

    /**
     * @brief Destroys the Trapezoidal Profile Generator.
     */
    ~TrapezoidalProfileGenerator() = default;

    /**
     * @brief Generates the motion profile for a specific distance.
     * 
     * Calculates the acceleration distance and peak reachable velocity based on the
     * target distance and configured constraints.
     * 
     * @param distance The total distance to travel.
     */
    void GenerateProfile(float distance) override;

    /**
     * @brief Calculates the target velocity at a specific progress point.
     * 
     * @param progress The current progress (displacement) from the start.
     * @return float The calculated target velocity.
     * 
     * @note The output velocity is clamped to a minimum magnitude of 10.0f.
     * @warning This method assumes GenerateProfile() has been called to initialize the profile.
     */
    float CalculateValue(float progress) override;

private:
    float _distance;
    float _acceleration;
    float _velocity;
    
    float _accelerationDistance;
    float _peakReachableVelocity;
};

} // namespace Motion::Core::Robot