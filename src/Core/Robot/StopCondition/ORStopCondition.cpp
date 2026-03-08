/**
 * @file ORStopCondition.cpp
 * @brief Implementation of the ORStopCondition class for applying OR logical operations on a group of stop conditions.
 */

#include "Motion/Core/Robot/StopCondition/ORStopCondition.h"

namespace Motion::Core::Robot {

ORStopCondition::ORStopCondition(BaseStopConditionHandle conditionA, BaseStopConditionHandle conditionB)
    : _conditionsA(conditionA), _conditionsB(conditionB) {}

ORStopCondition::~ORStopCondition() = default;

bool ORStopCondition::ShouldExit(float error)
{
    bool retA = _conditionsA->ShouldExit(error);
    bool retB = _conditionsB->ShouldExit(error);
    return retA || retB;
}

void ORStopCondition::Reset()
{
    _conditionsA->Reset();
    _conditionsB->Reset();
}

BaseStopConditionHandle operator||(BaseStopConditionHandle conditionA, BaseStopConditionHandle conditionB) {
    if (!conditionA || !conditionB) return nullptr;
    return BaseStopConditionHandle(new ORStopCondition(conditionA, conditionB));
}

} // namespace Motion::Core::Robot