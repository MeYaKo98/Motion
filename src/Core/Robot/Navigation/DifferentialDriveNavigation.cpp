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
    /// @todo Implement this function
}

void DifferentialDriveNavigation::Turn(float angle) {
    /// @todo Implement this function
}

void DifferentialDriveNavigation::MoveTo(float x, float y) {
    /// @todo Implement this function
}

void DifferentialDriveNavigation::Orient(float angle) {
    /// @todo Implement this function
}

}