/**
 * @file TolerenceStopCondition.cpp
 * @brief  A stop condition conditon that returns true if error is under tolerence value.
 */

#include "Core/Robot/StopCondition/TolerenceStopCondition.h"

namespace Motion::Core::Robot {

TolerenceStopCondition::TolerenceStopCondition(float tolerence) : _tolerence(abs(tolerence)) {}

TolerenceStopCondition::~TolerenceStopCondition() = default;

TolerenceStopConditionHandle TolerenceStopCondition::Create(float tolerence)
{
    if (tolerence == 0.0f) throw std::invalid_argument("Tolerence must not be zero");
    return TolerenceStopConditionHandle(new TolerenceStopCondition(tolerence));
}


bool TolerenceStopCondition::ShouldExit(float error) {
    if (abs(error)<_tolerence)
        return true;
    else
        return false;
}

void  TolerenceStopCondition::Reset() {}

}