/**
 * @file ToleranceStopCondition.cpp
 * @brief  A stop condition conditon that returns true if error is under tolerence value.
 */

#include "Core/Robot/StopCondition/ToleranceStopCondition.h"

namespace Motion::Core::Robot {

ToleranceStopCondition::ToleranceStopCondition(float tolerence) : _tolerance(abs(tolerence)) {}

ToleranceStopCondition::~ToleranceStopCondition() = default;

ToleranceStopConditionHandle ToleranceStopCondition::Create(float tolerence)
{
    if (tolerence == 0.0f) throw std::invalid_argument("Tolerence must not be zero");
    return ToleranceStopConditionHandle(new ToleranceStopCondition(tolerence));
}


bool ToleranceStopCondition::ShouldExit(float error) {
    if (abs(error)<_tolerance)
        return true;
    else
        return false;
}

void  ToleranceStopCondition::Reset() {}

}