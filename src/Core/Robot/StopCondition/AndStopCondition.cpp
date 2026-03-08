/**
 * @file AndStopCondition.cpp
 * @brief A class for applying AND logical operation on a group of stop condition.
 */

#include "Motion/Core/Robot/StopCondition/AndStopCondition.h"

namespace Motion::Core::Robot {

AndStopCondition::AndStopCondition(BaseStopConditionHandle conditionA, BaseStopConditionHandle conditionB)
    : _conditionsA(conditionA), _conditionsB(conditionB) {}

AndStopCondition::~AndStopCondition() = default;

bool AndStopCondition::ShouldExit(float error)
{
    bool retA = _conditionsA->ShouldExit(error);
    bool retB = _conditionsB->ShouldExit(error);
    return retA && retB;
}

void AndStopCondition::Reset()
{
    _conditionsA->Reset();
    _conditionsB->Reset();
}

BaseStopConditionHandle operator&&(BaseStopConditionHandle conditionA, BaseStopConditionHandle conditionB) {
    if (!conditionA || !conditionB) return nullptr;
    return BaseStopConditionHandle(new AndStopCondition(conditionA, conditionB));
}

} // namespace Motion::Core::Robot