/**
 * @file OscillationStopCondition.cpp
 * @brief Implementation of the OscillationStopCondition class.
 */

#include "Motion/Core/Robot/StopCondition/OscillationStopCondition.h"

namespace Motion::Core::Robot {

OscillationStopCondition::OscillationStopCondition(uint32_t targetOscillations)
    : _targetOscillations(targetOscillations),
      _currentOscillationCount(0),
      _previousError(0) {}

OscillationStopCondition::~OscillationStopCondition() = default;

OscillationStopConditionHandle OscillationStopCondition::Create(uint32_t targetOscillations)
{
    if (targetOscillations == 0) throw std::invalid_argument("Target oscillations must not be zero");
     
    return OscillationStopConditionHandle(new OscillationStopCondition(targetOscillations));
}

bool OscillationStopCondition::ShouldExit(float error) {
    // Detect oscillation (sign change) when both values are within tolerance
    if ((error > 0 && _previousError < 0) || (error < 0 && _previousError > 0)) {
            _currentOscillationCount++;
    }

    // Update previous error for next call
    _previousError = error;

    // Return true if we've reached target oscillation count AND error is in tolerance
    if (_currentOscillationCount >= _targetOscillations) {
        return true;
    }

    return false;
}

void OscillationStopCondition::Reset() {
    _currentOscillationCount = 0;
    _previousError = 0;
}

} // namespace Motion::Core::Robot
