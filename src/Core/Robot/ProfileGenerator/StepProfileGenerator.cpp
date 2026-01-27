/**
 * @file StepProfileGenerator.cpp
 * @brief A speed profile generator that returns constant speed.
 */

#include "Core\Robot\ProfileGenerator\StepProfileGenerator.h"

namespace Motion::Core::Robot {

StepProfileGenerator::StepProfileGenerator(float speed) : _speed(abs(speed)) {}

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