/**
 * @file DifferentialDriveNavigation.cpp
 * @brief A Navigation Class for differential drive robot.
 */

#include "Motion/Core/Robot/Navigation/DifferentialDriveNavigation.h"

namespace Motion::Core::Robot {

DifferentialDriveNavigation::DifferentialDriveNavigation (GenericDifferentialDriveOdometryHandle odometryHandle, BaseProfileGeneratorHandle profileGeneratorHandle, BaseStopConditionHandle stopConditionHandle, DifferentialDriveMotorConfig motorConfig)
    : GenericDifferentialDriveNavigation(odometryHandle, profileGeneratorHandle, stopConditionHandle, motorConfig) {}

DifferentialDriveNavigation::~DifferentialDriveNavigation() {}

DifferentialDriveNavigationHandle DifferentialDriveNavigation::Create(GenericDifferentialDriveOdometryHandle odometryHandle, BaseProfileGeneratorHandle profileGeneratorHandle, BaseStopConditionHandle stopConditionHandle, DifferentialDriveMotorConfig motorConfig)
{
    if (odometryHandle == nullptr) throw std::invalid_argument("odometryHandle cannot be nullptr");
    if (profileGeneratorHandle == nullptr) throw std::invalid_argument("profileGeneratorHandle cannot be nullptr");
    if (stopConditionHandle == nullptr) throw std::invalid_argument("stopConditionHandle cannot be nullptr");
    if (motorConfig.rightMotorHandle == nullptr) throw std::invalid_argument("rightMotorHandle cannot be nullptr");
    if (motorConfig.leftMotorHandle == nullptr) throw std::invalid_argument("leftMotorHandle cannot be nullptr");
    if (motorConfig.rightControllerHandle == nullptr) throw std::invalid_argument("rightController cannot be nullptr");
    if (motorConfig.leftControllerHandle == nullptr) throw std::invalid_argument("leftController cannot be nullptr");
    return DifferentialDriveNavigationHandle(new DifferentialDriveNavigation(odometryHandle, profileGeneratorHandle, stopConditionHandle, motorConfig));
}

void DifferentialDriveNavigation::Move(float distance)
{
    LOG_TRACE("Move called with distance: %f", distance);
    /// @todo Implement this function
}

void DifferentialDriveNavigation::Turn(float angle)
{
    LOG_TRACE("Turn called with angle: %f", angle);
    /// @todo Implement this function
}

void DifferentialDriveNavigation::MoveTo(float x, float y)
{
    LOG_TRACE("MoveTo called with x: %f, y: %f", x, y);
    /// @todo Implement this function
}

void DifferentialDriveNavigation::Orient(float angle)
{
    LOG_TRACE("Orient called with angle: %f", angle);
    /// @todo Implement this function
}

}