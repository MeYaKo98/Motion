/**
 * @file ORStopCondition.cpp
 * @brief Implementation of the ORStopCondition class for applying OR logical operations on a group of stop conditions.
 */

#include "Core\Robot\StopCondition\ORStopCondition.h"

namespace Motion::Core::Robot {

ORStopCondition::~ORStopCondition() = default;

bool ORStopCondition::ShouldExit(float error) {
    for (auto* cond : _conditions) {
        if (cond->ShouldExit(error)) return true;
    }
    return false;
}

void ORStopCondition::Reset() {
    for (auto* cond : _conditions) {
        cond->Reset();
    }
}

} // namespace Motion::Core::Robot