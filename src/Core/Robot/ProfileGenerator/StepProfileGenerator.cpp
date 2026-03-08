/**
 * @file StepProfileGenerator.cpp
 * @brief A speed profile generator that returns constant speed.
 */

#include "Motion/Core/Robot/ProfileGenerator/StepProfileGenerator.h"

namespace Motion::Core::Robot {

StepProfileGenerator::StepProfileGenerator(float speed) : _speed(abs(speed)) {}

StepProfileGeneratorHandle StepProfileGenerator::Create(float speed)
{
    if (speed == 0.0) throw std::invalid_argument("Speed cannot be zero");
    return StepProfileGeneratorHandle(new StepProfileGenerator(speed));
}

StepProfileGenerator::~StepProfileGenerator() = default;

void StepProfileGenerator::GenerateProfile(float distance) {
    _distance = distance;
}

float StepProfileGenerator::CalculateValue(float progress) {
    if (progress<_distance)
        return _speed;
    else
        return -_speed;
}

}