/**
 * @file BaseNavigation.h
 * @brief Defines the abstract base interface for robot navigation strategies.
 * @details This file contains the BaseNavigation class, which serves as a contract for
 *          implementing various navigation behaviors (e.g., differential drive, omnidirectional,
 *          holonomic). It abstracts away drive kinematics and focuses on high-level motion commands.
 */

#pragma once

#include <memory>
#include "Motion/Core/Robot/ProfileGenerator/BaseProfileGenerator.h"
#include "Motion/Core/Robot/StopCondition/BaseStopCondition.h"
#include "Motion/Core/Logger.h"

namespace Motion::Core::Robot {

#define NavigationPointer(T) std::unique_ptr<T>

/**
 * @brief An abstract interface for navigation systems across different drive types.
 * @details The BaseNavigation class standardizes high-level movement commands such as
 *          moving straight, turning, and moving to specific coordinates. It relies on
 *          injected dependencies for motion profiling and stop conditions.
 *
 *          Derived classes implement drive-specific kinematics:
 *          - DifferentialDriveNavigation: two wheels with independent speeds
 *          - OmnidirectionalNavigation: three or more wheels at angles
 *          - HolonomicNavigation: translation and rotation without constraints
 *
 * @note **Architecture:** Uses dependency injection for BaseProfileGenerator and BaseStopCondition
 *       to allow flexible configuration of motion profiles and stopping behavior.
 */
class BaseNavigation {
public:
    /**
     * @brief Virtual destructor for the BaseNavigation object.
     * @details Default implementation. Ensures proper cleanup of derived classes.
     *          Must be virtual to allow polymorphic deletion through base class pointers.
     */
    virtual ~BaseNavigation() = default;

    /**
     * @brief Moves the robot by a specified distance relative to its current position.
     * @details This method commands the robot to travel forward (positive distance) or
     *          backward (negative distance) along its current heading. The actual motion
     *          is controlled by the profile generator and stop condition.
     *
     *          The motion sequence is:
     *          1. Generate velocity profile based on distance
     *          2. Execute motion using closed-loop control (via controllers and odometry)
     *          3. Monitor stop condition until motion is complete
     *          4. Return when stop condition is satisfied or motion completes
     *
     * @param distance The distance to move. Positive values move forward; negative values move backward.
     *                 Units depend on the implementation (typically meters or cm).
     *                 Must be in the same units as the wheel radius.
     *
     * @note **Units:** The unit of `distance` is determined by the encoder resolution and wheel radius
     *       used in the odometry system. Ensure consistency across all components.
     *
     * @note **Blocking:** This is typically a blocking call that returns only when the motion
     *       completes. If non-blocking behavior is desired, call this method from a dedicated task.
     *
     * @note **Repeatability:** Multiple calls with the same distance should produce similar motion,
     *       assuming the robot's velocity/acceleration constraints remain constant.
     *
     * @warning **Zero Distance:** Calling with distance == 0 may cause the stop condition to trigger
     *          immediately, resulting in no motion. Some implementations may skip the motion entirely.
     *
     * @warning **Large Distances:** Very large distances may accumulate odometry error over time.
     *          For long-distance moves, consider using intermediate waypoints or external localization.
     *
     * @warning **Obstacle Collision:** This method does not check for obstacles. Ensure the path is clear
     *          before calling. Collisions will cause the robot to stall and may trigger timeout errors.
     */
    virtual void Move(float distance) = 0;

    /**
     * @brief Turns the robot by a specified angle relative to its current heading.
     * @details Rotates the robot in place by the specified angle. The direction of rotation
     *          depends on the sign: positive typically means counter-clockwise (CCW), negative clockwise (CW),
     *          but this is determined by the coordinate system and motor controller conventions.
     *
     *          The motion sequence is:
     *          1. Generate velocity profile (typically for rotational velocity)
     *          2. Execute rotation using differential motor control
     *          3. Monitor stop condition until rotation is complete
     *          4. Return when rotation reaches the target angle
     *
     * @param angle The relative angle to turn.
     *              Positive values rotate counter-clockwise (CCW) by default.
     *              Negative values rotate clockwise (CW) by default.
     *              Units depend on the implementation (typically radians).
     *
     * @note **Coordinate System:** The sign convention for positive rotation depends on:
     *       - The coordinate frame convention (ROS uses counter-clockwise as positive)
     *       - The motor controller wiring (reversed wiring reverses the rotation direction)
     *       Document the convention used in your implementation.
     *
     * @note **Units:** Typically radians in SI systems.
     *       Ensure consistency with the odometry system.
     *
     * @note **Yaw Rotation:** In 2D navigation, this rotates around the Z-axis (yaw rotations),
     *       which is equivalent to changing the robot's heading or theta.
     *
     * @note **In-Place Rotation:** This method rotates about the center of the robot without
     *       translating. For differential drives, wheels move in opposite directions.
     *
     * @note **Blocking:** Similar to Move(), this is typically blocking. Non-blocking calls
     *       should be made from separate tasks.
     *
     * @warning **Gyro Drift:** Without external heading feedback (e.g., magnetometer, IMU),
     *          rotation angles may drift due to wheel slip or odometry errors. Large rotation angles
     *          accumulate significant errors. Periodically correct with absolute heading sensors.
     *
     * @warning **Zero Angle:** Calling with angle == 0 may cause the stop condition to trigger
     *          immediately with no rotation. Some implementations may skip the motion.
     *
     * @see Orient() for absolute rotation to a specific heading.
     */
    virtual void Turn(float angle) = 0;

    /** 
     * @brief Moves the robot to a specific absolute coordinate on the map.
     * @details This method calculates the necessary translation and rotation to reach the target
     *          (x, y) point in the global coordinate frame. The path taken depends on the
     *          implementation strategy:
     *          - Turn & Move: Rotate to face target, then move straight
     *          - Circular Arc: Move along a curved path
     *          - Holonomic: Translate directly without changing heading first
     *
     *          General sequence:
     *          1. Calculate dx = x - current_x, dy = y - current_y
     *          2. Calculate target_distance = sqrt(dx² + dy²)
     *          3. Calculate target_heading = atan2(dy, dx)
     *          4. Rotate to target_heading if necessary
     *          5. Move forward by target_distance
     *          6. Return when target is reached (within stop condition tolerance)
     *
     * @param x The target X coordinate in the global frame (world frame).
     *          Unit must match the distance units used in the odometry system.
     * @param y The target Y coordinate in the global frame (world frame).
     *          Unit must match the distance units used in the odometry system.
     *
     * @note **Coordinate Frame:** Assumes a standard 2D Cartesian frame with X right and Y forward
     *       (or as defined by the odometry system). Document the frame convention in your implementation.
     *
     * @note **Path Planning:** This method does not perform path planning around obstacles.
     *       It assumes a straight line to the target is collision-free.
     *
     * @note **Absolute Positioning:** Uses global coordinates, which requires accurate odometry
     *       and periodic corrections from external sensors (e.g., vision, markers, GNSS).
     *
     * @note **Incremental Motion:** For long distances, consider breaking into multiple `MoveTo()` calls
     *       or waypoints to limit accumulated odometry error.
     *
     * @warning **Odometry Accumulation:** Odometry error accumulates over time. Long sequences of moves
     *          cause drift. Periodically reset position using absolute localization (e.g., vision...).
     *
     * @warning **Target Unreachable:** If an obstacle is in the way, the robot will stall and possibly
     *          exceed the stop condition timeout. Always verify the path is clear before calling.
     *
     * @see Move() for relative motion
     * @see Orient() for absolute heading rotation
     */
    virtual void MoveTo(float x, float y) = 0;

    /** 
     * @brief Orients the robot to a specific absolute angle in the global frame.
     * @details Rotates the robot so that its heading (theta) matches the specified absolute angle.
     *          Unlike `Turn()` which is relative, this is an absolute rotation.
     *
     *          The algorithm:
     *          1. Calculate angle_error = target_angle - current_heading
     *          2. Normalize angle_error to [-π, π] to take the shortest rotation path
     *          3. Call Turn(angle_error) to rotate
     *          4. Return when rotation is complete
     *
     * @param angle The desired absolute orientation angle in the global frame.
     *              Unit depends on the implementation (typically radians, range [0, 2π] or [-π, π]).
     *              Angle = 0 typically means facing along the positive X-axis.
     *              Angle = π/2 means facing along the positive Y-axis (counter-clockwise).
     *
     * @note **Angle Normalization:** This method should take the shortest rotation path.
     *       For example, rotating from 10° to 350° should rotate -20° (clockwise) not +340°.
     *       Implementations should normalize angle errors to [-π, π].
     *
     * @note **Units:** Ensure the angle unit matches the convention used in the odometry and Turn() methods.
     *
     * @note **Repeatability:** Calling Orient() with the same angle multiple times should produce
     *       the same heading (within sensor accuracy limits).
     *
     * @note **External Heading Reference:** For accurate orientation, use external sensors
     *       (e.g., magnetometer, IMU) to correct gyro drift.
     *
     * @warning **Heading Drift:** Without external heading feedback, repeated Orient() calls may
     *          fail to achieve the exact target due to accumulated gyro/odometry drift.
     *
     * @see Turn() for relative rotation
     * @see MoveTo() for moving to a target position
     */
    virtual void Orient(float angle) = 0;

protected:
    /**
     * @brief Constructs a new BaseNavigation object with required dependencies.
     * @details Initializes the navigation system with a specific motion profile generator
     *          and a stop condition. These are dependencies injected at construction time.
     *
     * @param profileGeneratorHandle A handle to a valid BaseProfileGenerator instance used for
     *                               calculating velocity curves (e.g., trapezoidal, S-curve).
     *                               The profile generator determines acceleration/deceleration behavior.
     *                               Must not be null. Ownership remains with the caller.
     * @param stopConditionHandle A handle to a valid BaseStopCondition instance used to determine
     *                           when a motion is complete (e.g., tolerance-based, time-based).
     *                           Must not be null. Ownership remains with the caller.
     *
     * @post `_profileGeneratorHandle` and `_stopConditionHandle` are initialized and ready for use.
     *
     * @warning The caller must ensure that the pointers passed are valid and remain valid
     *          for the lifetime of this object. If the dependencies are deleted externally,
     *          accessing them through the handles will cause undefined behavior (use-after-free).
     *
     * @warning Passing nullptr for either parameter will cause a crash when the navigation
     *          system attempts to use them. Validate inputs in derived Create() methods.
     */
    BaseNavigation(BaseProfileGeneratorHandle profileGeneratorHandle, BaseStopConditionHandle stopConditionHandle)
    : _profileGeneratorHandle(profileGeneratorHandle), _stopConditionHandle(stopConditionHandle) {}

    /**
     * @brief Pointer to the motion profile generator.
     * @details Used by derived classes to calculate velocity setpoints over time or distance.
     *          Responsible for generating smooth acceleration profiles to achieve the commanded distance
     *          or angle smoothly and safely.
     *
     *          Typical usage in derived classes:
     *          @code
     *          _profileGeneratorHandle->GenerateProfile(distance);
     *          float velocity = _profileGeneratorHandle->CalculateValue(progress);
     *          @endcode
     *
     * @note **Ownership:** The caller (typically the factory Create() method) retains ownership.
     *       This navigation instance only holds a reference.
     *
     * @warning **Null Check:** Always verify this is not nullptr before dereferencing.
     *          A nullptr indicates a programming error in the construction/factory method.
     */
    BaseProfileGeneratorHandle _profileGeneratorHandle;

    /**
     * @brief Pointer to the stop condition.
     * @details Used by derived classes to check if the target has been reached within the
     *          acceptable error margin. The stop condition is polled repeatedly during motion
     *          to determine when to exit the movement loop.
     *
     *          Typical usage in derived classes:
     *          @code
     *          while (!_stopConditionHandle->ShouldExit(error))
     *          {
     *              // Execute control loop iteration
     *          }
     *          @endcode
     *
     * @note **Ownership:** The caller retains ownership. This navigation instance holds a reference.
     * @warning **Null Check:** Always verify this is not nullptr before dereferencing.
     */
    BaseStopConditionHandle _stopConditionHandle;
};

using BaseNavigationHandle = NavigationPointer(BaseNavigation);

} // namespace Motion::Core::Robot