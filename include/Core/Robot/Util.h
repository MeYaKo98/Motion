/**
 * @file Util.h
 * @brief Defines utility structures and types used throughout the robot motion framework.
 */

#pragma once

namespace Motion::Core::Robot {

/**
 * @brief Represents a 2D pose or position of the robot in a Cartesian coordinate system.
 * @details This structure encapsulates the state of a robot in a 2D plane, consisting of its
 *          linear position (x, y) and its angular orientation (theta).
 */
struct Position {
    /**
     * @brief The X-coordinate of the robot's position.
     * @note The unit of measurement is defined by the specific implementation context (e.g., meters, millimeters).
     */
    float x;

    /**
     * @brief The Y-coordinate of the robot's position.
     * @note The unit of measurement is defined by the specific implementation context (e.g., meters, millimeters).
     */
    float y;

    /**
     * @brief The angular orientation (heading) of the robot.
     * @note The unit is typically radians. Positive values usually indicate a counter-clockwise rotation
     *       relative to the positive X-axis, but this depends on the coordinate system convention.
     */
    float theta;
};

} // namespace Motion::Core::Robot