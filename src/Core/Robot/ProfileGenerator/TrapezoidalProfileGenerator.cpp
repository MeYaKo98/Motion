#include "Motion/Core/Robot/ProfileGenerator/TrapezoidalProfileGenerator.h"
#include <cmath>

namespace Motion::Core::Robot {

TrapezoidalProfileGenerator::TrapezoidalProfileGenerator(float acceleration, float velocity, float minVelocity)
    : _acceleration(abs(acceleration)), _velocity(abs(velocity)), _minVelocity(abs(minVelocity)) {}

TrapezoidalProfileGeneratorHandle TrapezoidalProfileGenerator::Create(float acceleration, float velocity, float minVelocity) {
    if (acceleration == 0.0f) throw std::invalid_argument("Acceleration and velocity cannot be zero");
    if (velocity == 0.0f) throw std::invalid_argument("Velocity cannot be zero");
    if (minVelocity == 0.0f) throw std::invalid_argument("Minimum velocity cannot be zero");
    return TrapezoidalProfileGeneratorHandle(new TrapezoidalProfileGenerator(acceleration, velocity, minVelocity));
}

void TrapezoidalProfileGenerator::GenerateProfile(float distance) {
    _distance = distance;
    
    float absoluteDistance = std::abs(distance);
    _accelerationDistance = (_velocity * _velocity) / (2.0f * _acceleration);
    
    if (_accelerationDistance * 2.0f > absoluteDistance) {
        // Triangle profile
        _accelerationDistance = absoluteDistance / 2.0f;
        _peakReachableVelocity = std::sqrt(2.0f * _acceleration * _accelerationDistance);
    } else {
        // Trapezoidal profile
        _peakReachableVelocity = _velocity;
    }
}

float TrapezoidalProfileGenerator::CalculateValue(float progress) {
    float sign = ((_distance - progress) >= 0) ? 1.0f : -1.0f;
    
    float absoluteProgress = std::abs(progress);
    float absoluteDistance = std::abs(_distance);
    float absoluteRemaining = std::abs(absoluteDistance - absoluteProgress);

    float outputVelocity;
    // Phase 1: Acceleration
    if (absoluteProgress < _accelerationDistance) {
        outputVelocity = std::sqrt(2.0f * _acceleration * absoluteProgress);
    }
    // Phase 2: Cruise
    else if (absoluteRemaining > _accelerationDistance) {
        outputVelocity = _peakReachableVelocity;
    }
    // Phase 3: Deceleration
    else {
        outputVelocity = std::sqrt(2.0f * _acceleration * absoluteRemaining);
    }

    if (outputVelocity < _minVelocity) {
        outputVelocity = _minVelocity;
    }

    return outputVelocity * sign;
}

} // namespace Motion::Core::Robot