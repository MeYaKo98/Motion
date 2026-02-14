/**
 * @file AndStopCondiiton.cpp
 * @brief A class for applying AND logical operation on a group of stop condition.
 */

#include "Core/Robot/StopCondition/AndStopCondition.h"

namespace Motion::Core::Robot {

AndStopCondition::~AndStopCondition() = default;

bool AndStopCondition::ShouldExit(float error) {
    for (auto* cond : _conditions) {
        if (!cond->ShouldExit(error)) return false;
    }
    return true;
}

void AndStopCondition::Reset() {
    for (auto* cond : _conditions) {
        cond->Reset();
    }
}

} // namespace Motion::Core::Robot