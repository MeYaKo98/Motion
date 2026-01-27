/**
 * @file TolarenceStopCondition.cpp
 * @brief  A stop condition conditon that returns true if error is under tolarence value.
 */

#include "Core\Robot\StopCondition\TolarenceStopCondition.h"

namespace Motion::Core::Robot {

TolarenceStopCondition::TolarenceStopCondition(float tolerence) : _tolarence(abs(tolerence)) {}

TolarenceStopCondition::~TolarenceStopCondition() = default;

bool TolarenceStopCondition::ShouldExit(float error) {
    if (abs(error)<_tolarence)
        return true;
    else
        return false;
}

void  TolarenceStopCondition::Reset() {}

}