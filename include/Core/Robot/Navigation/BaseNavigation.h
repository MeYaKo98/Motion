/**
 * @file BaseNavigation.h
 * @brief Defines the abstract base interface for robot navigation strategies.
 * @details This file contains the BaseNavigation class, which serves as a contract for
 *          implementing various navigation behaviors (e.g., differential drive, omnidirectional).
 */

#pragma once

#include "Core/Robot/ProfileGenerator/BaseProfileGenerator.h"
#include "Core/Robot/StopCondition/BaseStopCondition.h"
#include "Core/Diagnostics/Logger.h"

namespace Motion::Core::Robot {

/**
 * @brief An abstract interface for navigation systems across different drive types.
 * @details The BaseNavigation class standardizes high-level movement commands such as
 *          moving straight, turning, and moving to specific coordinates. It relies on
 *          injected dependencies for motion profiling and stop conditions.
 */
class BaseNavigation {
public:
    /**
     * @brief Constructs a new BaseNavigation object with required dependencies.
     * @details Initializes the navigation system with a specific motion profile generator
     *          and a stop condition.
     * @param profileGenerator A pointer to a valid BaseProfileGenerator instance used for
     *                         calculating velocity curves.
     * @param stopCondition A pointer to a valid BaseStopCondition instance used to determine
     *                      when a motion is complete.
     * @warning The caller must ensure that the pointers passed are valid and remain valid
     *          for the lifetime of this object. Passing nullptr may result in undefined behavior.
     */
    BaseNavigation(BaseProfileGenerator* profileGenerator, BaseStopCondition* stopCondition) : _profileGenerator(profileGenerator), _stopCondition(stopCondition) {
        if (_stopCondition == nullptr) LOG_ERROR("StopCondition can not be NULL");
        if (_profileGenerator == nullptr) LOG_ERROR("ProfileGenerator can not be NULL");
    };

    /**
     * @brief Virtual destructor for the BaseNavigation object.
     * @details Default implementation. Ensures proper cleanup of derived classes.
     */
    virtual ~BaseNavigation() = default;

    /**
     * @brief Moves the robot by a specified distance relative to its current position.
     * @details This method commands the robot to travel forward (positive distance) or
     *          backward (negative distance) along its current heading.
     * @param distance The distance to move. The unit depends on the implementation.
     */
    virtual void Move(float distance) = 0;

    /**
     * @brief Turns the robot by a specified angle relative to its current heading.
     * @details Rotates the robot in place. The direction of rotation for positive values
     *          depends on the specific coordinate system used by the implementation.
     * @param angle The relative angle to turn. The unit depends on the implementation (typically radians).
     */
    virtual void Turn(float angle) = 0;

    /** 
     * @brief Moves the robot to a specific absolute coordinate on the map.
     * @details This method calculates the necessary translation and rotation to reach the target (x, y) point.
     * @param x The target X coordinate in the global frame.
     * @param y The target Y coordinate in the global frame.
     */
    virtual void MoveTo(float x, float y) = 0;

    /** 
     * @brief Orients the robot to a specific absolute angle in the global frame.
     * @details Rotates the robot so that its heading matches the specified absolute angle.
     * @param angle The desired absolute orientation angle.
     */
    virtual void Orient(float angle) = 0;

private:


protected:
    /**
     * @brief Pointer to the motion profile generator.
     * @details Used by derived classes to calculate velocity setpoints over time or distance.
     */
    BaseProfileGenerator* _profileGenerator;

    /**
     * @brief Pointer to the stop condition.
     * @details Used by derived classes to check if the target has been reached within the acceptable error margin.
     */
    BaseStopCondition* _stopCondition;
};

} // namespace Motion::Core::Robot