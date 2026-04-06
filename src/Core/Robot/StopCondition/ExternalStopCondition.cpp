/**
 * @file ExternalStopCondition.cpp
 * @brief Implementation of the ExternalStopCondition class.
 */

#include "Motion/Core/Robot/StopCondition/ExternalStopCondition.h"

namespace Motion::Core::Robot {

ExternalStopConditionHandle ExternalStopCondition::Create()
{
    return ExternalStopConditionHandle(new ExternalStopCondition());
}

ExternalStopCondition::ExternalStopCondition() : _exitFlag(false) {}

ExternalStopCondition::~ExternalStopCondition() {}

bool ExternalStopCondition::ShouldExit(float error) 
{
    (void)error; // Suppress unused parameter warning
    return _exitFlag.load();
}

void ExternalStopCondition::Reset() 
{
    _exitFlag.store(false);
}

void ExternalStopCondition::SetFlag(bool shouldExit) 
{
    _exitFlag.store(shouldExit);
}

} // namespace Motion::Core::Robot
