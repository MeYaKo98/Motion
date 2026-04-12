/**
 * @file SettleStopCondition.cpp
 * @brief Implementation of the SettleStopCondition class.
 */

#include "Motion/Core/Robot/StopCondition/SettleStopCondition.h"

namespace Motion::Core::Robot {

SettleStopCondition::SettleStopCondition(float tolerance, uint32_t targetSettleTime)
    : _tolerance(std::abs(tolerance)), 
      _targetSettleTime(targetSettleTime),
      _settleStartTime(0),
      _isSettling(false) {}

SettleStopCondition::~SettleStopCondition() = default;

SettleStopConditionHandle SettleStopCondition::Create(float tolerance, uint32_t targetSettleTime)
{
    if (tolerance == 0.0f) throw std::invalid_argument("Tolerance must not be zero");
    if (targetSettleTime == 0) throw std::invalid_argument("Target settle time must not be zero");
    
    return SettleStopConditionHandle(new SettleStopCondition(tolerance, targetSettleTime));
}

bool SettleStopCondition::ShouldExit(float error) {
    bool inTolerance = std::abs(error) <= _tolerance;

    if (!inTolerance) {
        // Error is outside tolerance, reset settling state
        _isSettling = false;
        _settleStartTime = 0;
        return false;
    }

    // Error is within tolerance
    if (!_isSettling) {
        // First time entering tolerance, start the settle timer
        _settleStartTime = xTaskGetTickCount();
        _isSettling = true;
        return false;
    }

    // We are already settling, check if settle time has elapsed
    TickType_t elapsedTicks = xTaskGetTickCount() - _settleStartTime;
    uint32_t elapsedMs = pdTICKS_TO_MS(elapsedTicks);

    if (elapsedMs >= _targetSettleTime) {
        return true;  // Settle time complete, ready to exit
    }

    return false;  // Still settling, not yet ready to exit
}

void SettleStopCondition::Reset() {
    _isSettling = false;
    _settleStartTime = 0;
}

} // namespace Motion::Core::Robot
