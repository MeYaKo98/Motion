/**
 * @file DifferentialDriveNavigation.cpp
 * @brief A Navigation Class for differential drive robot.
 */

#include "Core\Robot\Navigation\DifferentialDriveNavigation.h"

namespace Motion::Core::Robot {

DifferentialDriveNavigation::DifferentialDriveNavigation(GenericDifferentialDriveOdometry* odometry, BaseProfileGenerator* profileGenerator, BaseStopCondition* stopCondition, DifferentialDriveMotorConfig motorConfig)
    : GenericDifferentialDriveNavigation(odometry, profileGenerator, stopCondition, motorConfig) {}

DifferentialDriveNavigation::~DifferentialDriveNavigation() {}

void DifferentialDriveNavigation::Move(float distance) {
    LOG_TRACE("Move called with distance: %f", distance);
    /// @todo Implement this function
}

void DifferentialDriveNavigation::Turn(float angle) {
    LOG_TRACE("Turn called with angle: %f", angle);
    /// @todo Implement this function
}

void DifferentialDriveNavigation::MoveTo(float x, float y) {
    LOG_TRACE("MoveTo called with x: %f, y: %f", x, y);
    /// @todo Implement this function
}

void DifferentialDriveNavigation::Orient(float angle) {
    LOG_TRACE("Orient called with angle: %f", angle);
    /// @todo Implement this function
}

}