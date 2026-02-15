/**
 * @file DifferentialDriveNavigation.h
 * @brief A navigation class for differential drive robot.
 */

#pragma once

#include "Core/Robot/Navigation/GenericDifferentialDriveNavigation.h"

namespace Motion::Core::Robot {

/**
 * @brief A concrete navigation class for differential drive robots.
 * @details This class implements navigation primitives (Move, Turn, MoveTo, Orient) specifically
 *          for differential drive kinematics. It orchestrates the odometry, profile generation,
 *          and motor control to execute motion commands.
 */
class DifferentialDriveNavigation;

using DifferentialDriveNavigationHandle = NavigationPointer(DifferentialDriveNavigation);

class DifferentialDriveNavigation : public GenericDifferentialDriveNavigation{
public:

    static DifferentialDriveNavigationHandle Create(GenericDifferentialDriveOdometryHandle odometryHandle, BaseProfileGeneratorHandle profileGeneratorHandle, BaseStopConditionHandle stopConditionHandle, DifferentialDriveMotorConfig motorConfig);

    /**
     * @brief Destructor of the DifferentialDriveNavigation object.
     */
    ~DifferentialDriveNavigation();

    /**
     * @brief Moves the robot by a certain distance in the direction it is facing.
     * @param distance The distance to move in meters. Positive values move forward, negative values move backward.
     */
    void Move(float distance) override;

    /**
     * @brief Turns the robot by a certain angle in place.
     * @param angle The angle to turn in radians. Positive values typically indicate counter-clockwise rotation.
     */
    void Turn(float angle) override;

    /** 
     * @brief Moves the robot to a certain position on the map.
     * @details This method handles the path planning (usually rotation then translation) to reach the target.
     * @param x The target X coordinate in the global frame.
     * @param y The target Y coordinate in the global frame.
     */
    void MoveTo(float x, float y) override;

    /** 
     * @brief Orients the robot to a specific absolute angle on the map.
     * @details Rotates the robot to match the target heading.
     * @param angle The desired absolute orientation angle in radians.
     */
    void Orient(float angle) override;

protected:
    /**
     * @brief Constructs a new DifferentialDriveNavigation object.
     * @details Initializes the navigation system with the required components.
     * @param odometry Pointer to the odometry instance for position tracking. Must not be nullptr.
     * @param profileGenerator Pointer to the profile generator for velocity planning. Must not be nullptr.
     * @param stopCondition Pointer to the stop condition logic. Must not be nullptr.
     * @param motorConfig Configuration structure containing motor parameters.
     * @warning The caller is responsible for ensuring that the pointers passed (odometry, profileGenerator, stopCondition)
     *          remain valid for the lifetime of this object.
     */
    explicit DifferentialDriveNavigation(GenericDifferentialDriveOdometryHandle odometryHandle, BaseProfileGeneratorHandle profileGeneratorHandle, BaseStopConditionHandle stopConditionHandle, DifferentialDriveMotorConfig motorConfig);
};

} // namespace Motion::Core::Robot