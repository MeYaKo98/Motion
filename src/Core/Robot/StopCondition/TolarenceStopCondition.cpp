/**
 * @file TolerenceStopCondition.cpp
 * @brief  A stop condition conditon that returns true if error is under tolerence value.
 */

#include "Core/Robot/StopCondition/TolerenceStopCondition.h"

namespace Motion::Core::Robot {

TolerenceStopCondition::TolerenceStopCondition(float tolerence) : _tolerence(abs(tolerence)) {
    if (_tolerence == 0.0) LOG_ERROR("Tolerence cannot be zero");
}

TolerenceStopCondition::~TolerenceStopCondition() = default;

bool TolerenceStopCondition::ShouldExit(float error) {
    if (abs(error)<_tolerence)
        return true;
    else
        return false;
}

void  TolerenceStopCondition::Reset() {}

}