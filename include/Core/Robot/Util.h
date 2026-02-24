/**
 * @file Util.h
 * @brief Defines utility structures and types used throughout the robot motion framework.
 * @details Contains fundamental data structures representing robot state and configuration.
 */

#pragma once

namespace Motion::Core::Robot {

/**
 * @brief Represents a 2D pose (position + orientation) of the robot in a Cartesian coordinate system.
 * @details This structure encapsulates the complete state of a robot in a 2D plane, consisting of its
 *          linear position (x, y) and its angular orientation (heading or theta).
 *          Used extensively in odometry, navigation, and state tracking systems.
 *
 *          **Coordinate System Convention:**
 *          By default, this framework assumes:
 *          - X-axis: Points to the right (positive direction)
 *          - Y-axis: Points forward (positive direction)
 *          - Theta: Measured counter-clockwise from the positive X-axis (right-hand rule)
 *          - Theta = 0: Robot facing along the positive X-axis (right)
 *          - Theta = π/2: Robot facing along the positive Y-axis (forward)
 *          - Theta = π: Robot facing along the negative X-axis (left)
 *          - Theta = -π/2: Robot facing along the negative Y-axis (backward)
 *
 *          This follows the standard ROS (Robot Operating System) convention.
 *
 *          **Physics Context:**
 *          In 2D robotics, the position and orientation combine to uniquely define the robot's
 *          state on a plane. This is fundamental for:
 *          - Odometry: Calculating position from wheel/sensor feedback
 *          - Navigation: Planning and executing motion to target positions
 *          - Localization: Determining where the robot is in the environment
 *          - Control: Commanding the robot to specific poses
 *
 *          **3D Extension:**
 *          For 3D systems, additional fields (z position, roll, pitch) would be needed.
 *          This structure is specifically for planar (2D) robots.
 *
 * @note **Units:** The structure does not enforce or define units. The framework must use
 *       consistent units across all components:
 *       - Position: Typically meters (m) or millimeters (mm)
 *       - Orientation: Typically radians (rad) or degrees (deg)
 *       Always document the units used in your application.
 *
 * @note **Initialization:** By default, creating a Position instance leaves members uninitialized.
 *       Always explicitly initialize: `Position pos = {0.0f, 0.0f, 0.0f};`
 *       Or use designated initializers: `Position pos{.x = 1.0f, .y = 2.0f, .theta = 0.0f};` (C++20)
 *
 * @note **POD Type:** This is a plain-old-data (POD) structure. It can be:
 *       - Safely copied with memcpy
 *       - Serialized for transmission
 *       - Stored in files or databases
 *       - Passed to memcmp for comparison
 *
 * @note **Memory Layout:** 3 × float = 12 bytes (on systems where float is 4 bytes).
 *       No padding is added by the compiler in normal circumstances.
 *
 * @see GenericDifferentialDriveOdometry::DifferentialDriveState for a related structure
 *      that includes wheel-specific state.
 */
struct Position {
    /**
     * @brief The X-coordinate of the robot's position in the global frame.
     * @details Represents the horizontal (left-right) position of the robot.
     *          Positive values typically mean "to the right"; negative mean "to the left".
     *          The exact interpretation depends on the coordinate frame convention.
     *
     * @note The unit of measurement is defined by the specific implementation context
     *       (e.g., meters, millimeters, feet). Must be consistent with y and the wheel radius.
     *
     * @note **Typical Usage:**
     *       - Initialized to 0.0f at robot startup
     *       - Updated by odometry at each update cycle
     *       - Compared to target x for navigation
     *       - Transmitted in telemetry/logging
     *
     * @note **Range:** Typically within [-1000, 1000] for room-sized robots
     *       (assuming meters as units). Larger ranges may require double precision
     *       to maintain accuracy.
     *
     * @warning **Precision Loss:** Over long distances, floating-point accumulation
     *          errors in odometry can cause this value to drift. Periodically correct
     *          using absolute localization (e.g., vision, markers, GNSS).
     */
    float x;

    /**
     * @brief The Y-coordinate of the robot's position in the global frame.
     * @details Represents the forward (depth) position of the robot.
     *          Positive values typically mean "forward"; negative mean "backward".
     *          The exact interpretation depends on the coordinate frame convention.
     *
     * @note The unit of measurement is defined by the specific implementation context
     *       (e.g., meters, millimeters). Must be consistent with x and wheel radius.
     *
     * @note **Typical Usage:**
     *       - Initialized to 0.0f at startup
     *       - Updated by odometry each cycle
     *       - Compared to target y for navigation
     *       - Used in trajectory planning
     *
     * @note **Range:** Similar to x, typically [-1000, 1000] for room-sized robots
     *       with meter units.
     *
     * @warning **Odometry Drift:** Cumulative errors in wheel encoders and odometry
     *          calculations cause y (and x) to drift over time. Drift is proportional
     *          to distance traveled. Implement periodic correction.
     */
    float y;

    /**
     * @brief The angular orientation (heading or yaw) of the robot.
     * @details Represents the direction the robot is facing in the global frame.
     *          Combined with x and y, fully specifies the robot's 2D pose (position + orientation).
     *
     *          **Angle Convention (Standard ROS):**
     *          - theta = 0: Robot facing along the positive X-axis (to the right)
     *          - theta = π/2: Robot facing along the positive Y-axis (forward)
     *          - theta = π (or -π): Robot facing along the negative X-axis (left)
     *          - theta = -π/2: Robot facing along the negative Y-axis (backward)
     *          - theta ∈ (-π, π]: Normalized range (use atan2 for calculations)
     *
     *          This follows the right-hand rule: curl fingers from X-axis toward Y-axis,
     *          thumb points in the direction of increasing angle (upward in 2D plane).
     *
     * @note **Units:** Typically radians (standard SI unit for angles).
     *       Some applications use degrees; always document your convention.
     *       Conversion: degrees = radians × 180/π, radians = degrees × π/180
     *
     * @note **Normalization:** The theta value is NOT automatically normalized to [-π, π].
     *       Application code should normalize when needed:
     *       ```cpp
     *       while (pos.theta > M_PI) pos.theta -= 2 * M_PI;
     *       while (pos.theta < -M_PI) pos.theta += 2 * M_PI;
     *       ```
     *       Or use: `theta = atan2(sin(theta), cos(theta));`
     *
     * @note **Typical Usage:**
     *       - Initialized to 0.0f (robot facing right/forward)
     *       - Updated by odometry each cycle from wheel velocities
     *       - Compared to target angle for rotation
     *       - Used in kinematic transformations
     *       - Transmitted to remote operators or localization systems
     *
     * @note **Accumulation:** Theta accumulates over time from the differential drive
     *       heading rate: dtheta/dt = (v_right - v_left) / wheel_spacing
     *
     * @warning **Gyro Drift:** Without external heading feedback (e.g., magnetometer, IMU),
     *          theta drifts due to wheel slip and odometry errors. The drift rate is typically
     *          1-5 degrees per meter of travel for consumer-grade encoders.
     *          Periodically correct using absolute orientation sensors.
     *
     * @warning **Wrapping:** When theta exceeds ±π, it wraps around. Code comparing angles
     *          must account for wraparound. Example: 170° and -170° are only 20° apart,
     *          not 340° apart. Always use angle difference functions:
     *          ```cpp
     *          float angleDiff = atan2(sin(target - current), cos(target - current));
     *          ```
     *
     * @warning **Large Accumulations:** If code doesn't periodically normalize theta,
     *          it can grow unbounded (e.g., achieving theta = 10π after 5 full rotations).
     *          This is mathematically equivalent to the normalized angle but may cause
     *          numerical precision issues or unexpected behavior in comparisons.
     *
     * @see BaseOdometry for the system that updates this value
     * @see DifferentialDriveOdometry for the differential drive implementation
     */
    float theta;
};

} // namespace Motion::Core::Robot