/**
 * @file DifferentialDriveNavigation.h
 * @brief Concrete navigation implementation for differential drive robots.
 * @details Implements high-level motion commands (Move, Turn, MoveTo, Orient) specifically
 *          for differential drive kinematics. Orchestrates odometry, profile generation,
 *          and motor/controller systems.
 */

#pragma once

#include "Core/Robot/Navigation/GenericDifferentialDriveNavigation.h"

namespace Motion::Core::Robot {

class DifferentialDriveNavigation;

using DifferentialDriveNavigationHandle = NavigationPointer(DifferentialDriveNavigation);

/**
 * @brief A concrete navigation system for differential drive robots.
 * @details This class implements the complete navigation logic for a differential drive robot,
 *          building on top of GenericDifferentialDriveNavigation. It orchestrates:
 *          - **Odometry:** Tracking robot position from wheel encoders
 *          - **Motion Profiling:** Generating smooth velocity commands respecting acceleration constraints
 *          - **Control Loop:** Using PID or other controllers to follow velocity profiles
 *          - **Motor Drive:** Commanding left and right motors independently
 *          - **Stop Conditions:** Determining when motion is complete
 *
 *          **Motion Execution Flow:**
 *          1. User calls Move(distance) or other motion command
 *          2. Profile generator creates smooth velocity profile for the distance
 *          3. Control loop repeatedly:  
 *             a. Reads current robot position from odometry  
 *             b. Calculates remaining distance/angle error  
 *             c. Generates control command via PID controller based on position error  
 *             d. Commands motors with computed command  
 *             e. Checks stop condition  
 *          4. When stop condition met, motion completes and function returns
 *
 *          **Differential Drive Kinematics:**
 *          Two independently-controlled wheels allow:
 *          - Straight motion: Both wheels at same speed
 *          - Turning: Wheels at different speeds
 *          - Spinning: Wheels at opposite speeds
 *
 * @note **Real-Time Performance:** The motion commands are blocking calls. For non-blocking
 *       behavior, call from separate FreeRTOS tasks or implement task-based motion scheduling.
 *
 * @note **Odometry Accuracy:** Motion accuracy depends on:
 *       - Encoder resolution (higher = better accuracy)
 *       - Wheel radius accuracy (measure carefully)
 *       - Wheel spacing accuracy (measure carefully)
 *       - Surface friction (loss on slippery surfaces)
 *       - Motor response linearity (non-linear response causes drift)
 *
 * @note **Tuning Required:** Controller gains must be tuned for your specific robot hardware.
 *
 * @see BaseNavigation for the abstract interface
 * @see GenericDifferentialDriveNavigation for the generic implementation
 * @see GenericDifferentialDriveOdometry for the odometry system
 * @see BaseController for the motion controller
 */
class DifferentialDriveNavigation : public GenericDifferentialDriveNavigation{
public:

    /**
     * @brief Factory method to create a DifferentialDriveNavigation instance.
     * @details Creates a new navigation system with full validation of all dependencies.
     *          Ensures the navigation system is properly configured before use.
     *
     * @param odometryHandle Smart pointer to a valid odometry system (e.g., DifferentialDriveOdometry).
     *                       Used for position tracking. Must not be nullptr.
     *
     * @param profileGeneratorHandle Smart pointer to a motion profile generator
     *                               (e.g., TrapezoidalProfileGenerator).
     *                               Used for smooth velocity commands. Must not be nullptr.
     *
     * @param stopConditionHandle Smart pointer to a stop condition (e.g., ToleranceStopCondition).
     *                           Used to detect motion completion. Must not be nullptr.
     *
     * @param motorConfig Configuration structure containing:
     *                   - rightMotorHandle: Motor for right wheel (must not be nullptr)
     *                   - leftMotorHandle: Motor for left wheel (must not be nullptr)
     *                   - rightControllerHandle: Controller for right wheel (must not be nullptr)
     *                   - leftControllerHandle: Controller for left wheel (must not be nullptr)
     *
     * @return DifferentialDriveNavigationHandle A smart pointer to the navigation instance.
     *
     * @throws std::invalid_argument if any handle is nullptr
     *
     * @note **Comprehensive Validation:** All parameters are checked to ensure no nullptr
     *       pointers are used. Invalid configuration throws an exception rather than
     *       silently failing at runtime.
     *
     * @note **Typical Usage:**
     *       @code
     *       auto odometry = DifferentialDriveOdometry::Create(wheelSpacing, rightWheel, leftWheel);
     *       auto profile = TrapezoidalProfileGenerator::Create(2.0f, 1.0f);
     *       auto stopCondition = ToleranceStopCondition::Create(0.01f);
     *       DifferentialDriveMotorConfig config = {rightMotor, leftMotor, rightCtrl, leftCtrl};
     *       auto nav = DifferentialDriveNavigation::Create(odometry, profile, stopCondition, config);
     *       @endcode
     *
     * @warning **Dependency Lifetime:** All passed handles must remain valid for the entire
     *          lifetime of the navigation system. If any dependency is deleted externally,
     *          using the navigation system causes undefined behavior.
     */
    static DifferentialDriveNavigationHandle Create(
        GenericDifferentialDriveOdometryHandle odometryHandle,
        BaseProfileGeneratorHandle profileGeneratorHandle,
        BaseStopConditionHandle stopConditionHandle,
        DifferentialDriveMotorConfig motorConfig);

    /**
     * @brief Destructor of the DifferentialDriveNavigation object.
     * @details Cleans up any allocated resources.
     *          The destructor does not delete the dependency objects (odometry, controllers, etc.);
     *          the creator/owner of those objects retains responsibility for cleanup.
     */
    ~DifferentialDriveNavigation();

    /**
     * @brief Moves the robot by a specified distance in the direction it is currently facing.
     * @details Commands the robot to travel forward (positive distance) or backward (negative distance)
     *          along its current heading. The motion is executed using the configured odometry,
     *          profile generator, and motor controllers to achieve smooth, controlled motion.
     *
     *          **Algorithm (Pseudo-code):**
     *          @code
     *          Set initial position from odometry
     *          Generate velocity profile for the distance
     *          While not at target:
     *              Current distance traveled = current_position - initial_position
     *              Remaining distance = target_distance - current_distance
     *              Velocity command = profile->CalculateValue(current_distance)
     *              Position error = remaining_distance
     *              Motor command = controller->GenerateCommand(command, current_velocity)
     *              SetCommand(right_motor, motor_command)
     *              SetCommand(left_motor, motor_command)
     *              If stopCondition->ShouldExit(position_error):
     *                  break
     *          Stop motors
     *          @endcode
     *
     * @param distance The distance to move.
     *                 - Positive values: Move forward (along current heading)
     *                 - Negative values: Move backward (opposite current heading)
     *                 - Units: Same as wheel radius (typically meters)
     *
     * @note **Relative Motion:** This command is relative to the robot's current position.
     *       Multiple calls accumulate: Move(1.0f) twice moves 2.0 total.
     *
     * @note **Blocking Operation:** This is a blocking call that returns only when the motion
     *       completes (stop condition met) or times out. For non-blocking operation, call from
     *       a separate FreeRTOS task.
     *
     * @note **Direction:** The direction is determined by the sign of distance:
     *       - Move(1.0f): Robot drives 1 meter forward
     *       - Move(-1.0f): Robot drives 1 meter backward
     *
     * @note **Speed Control:** The actual speed profile is determined by the ProfileGenerator
     *       configured for this navigation system. It respects acceleration and velocity limits.
     * 
     * @warning **Odometry Dependency:** Motion accuracy depends critically on odometry accuracy.
     *          Wheel slipping, encoder errors, or surface properties cause position drift.
     *          For long distances, consider using intermediate checkpoints or external localization.
     *
     * @warning **Obstacle Collision:** This method does not check for obstacles.
     *          Ensure the path is clear before calling. Collisions will cause the robot
     *          to stall against the obstacle (potentially damaging hardware). It is possible to
     *          incorporate the collision detection in a stop condition by the user.
     *
     * @warning **Zero Distance:** Calling Move(0.0f) may cause unexpected behavior
     *          (immediate return or no motion). Avoid zero distances.
     *
     * @see Turn() for relative rotation
     * @see MoveTo() for absolute positioning
     * @see BaseNavigation::Move()
     */
    void Move(float distance) override;

    /**
     * @brief Turns the robot by a specified angle relative to its current heading.
     * @details Commands the robot to rotate in place by the specified angle. Positive angles
     *          typically rotate counter-clockwise; negative rotate clockwise (depends on motor wiring).
     *          The rotation uses differential tank-style turning where wheels move in opposite directions.
     *
     *          **Algorithm Concept:**
     *          For a differential drive robot with wheel spacing L:
     *          Rotation = angle_rad × L / 2 = distance each wheel travels
     *
     *          **In-Place Rotation:**
     *          - Right wheel moves forward at speed v
     *          - Left wheel moves backward at speed -v
     *          - Robot spins at angular velocity ω = 2v / L
     *
     * @param angle The relative angle to turn.
     *              - Positive: Counter-clockwise rotation (depends on motor wiring convention)
     *              - Negative: Clockwise rotation
     *              - Units: Radians
     *
     * @note **Relative Rotation:** This command rotates relative to the robot's current heading.
     *       Multiple calls accumulate: Turn(π/2) twice rotates π radians total.
     *
     * @note **In-Place Spinning:** The robot rotates about its center point without translating.
     *       This is achieved by commanding wheels to move in opposite directions at equal speeds.
     *
     * @note **Blocking Operation:** This is a blocking call that returns when rotation completes.
     *       For non-blocking behavior, call from a separate task.
     *
     * @note **Sign Convention:** Positive angle typically means counter-clockwise rotation
     *       (standard mathematical convention), but this depends on how motors are wired.
     *       Always test your robot's turning direction!
     *
     * @note **Speed Profile:** The actual angular velocity profile uses the same motion profile
     *       generator configured for the navigation system, ensuring smooth acceleration.
     *
     * @warning **Gyro Drift:** Without external heading feedback (magnetometer, IMU),
     *          repeated rotation errors accumulate. For long operation, periodically correct
     *          heading using external sensors.
     *
     * @warning **Slipping:** On slippery floors, the robot may slip during rotation,
     *          causing actual rotation to differ from commanded rotation.
     *
     * @warning **Zero Angle:** Turning by zero radians may cause unexpected behavior (immediate return).
     *          Avoid zero angles.
     *
     * @warning **Large Angles:** Very large angles (> 10π) may wrap multiple times
     *          and accumulate significant odometry error. Consider using Orient() for
     *          absolute heading targets.
     *
     * @see Move() for forward/backward motion
     * @see Orient() for absolute heading rotation
     * @see BaseNavigation::Turn()
     */
    void Turn(float angle) override;

    /** 
     * @brief Moves the robot to a specific absolute coordinate on the global map.
     * @details Computes the necessary translation and rotation to reach the target (x, y)
     *          position in the global coordinate frame. Implements a "turn then move"
     *          strategy: first rotate to face the target, then move straight to it.
     *
     *          **Algorithm Concept:**
     *          @code
     *          delta_x = target_x - current_x
     *          delta_y = target_y - current_y
     *          target_distance = sqrt(delta_x² + delta_y²)
     *          target_heading = atan2(delta_y, delta_x)
     *          Orient(target_heading)     // Rotate to face target
     *          Move(target_distance)      // Move to target
     *          @endcode
     *
     * @param x The target X coordinate in the global (world) frame.
     *          Units: Same as wheel radius and odometry (typically meters).
     *
     * @param y The target Y coordinate in the global (world) frame.
     *          Units: Same as wheel radius and odometry (typically meters).
     *
     * @note **Absolute Positioning:** Unlike Move(), this command specifies an absolute
     *       position. The robot will attempt to reach exactly (x, y) regardless of where it started.
     *
     * @note **Path Planning:** The simplest implementation (turn then move) is used.
     *       More sophisticated path planning (curves, obstacle avoidance) would require
     *       additional components not present in this basic system.
     *
     * @note **Blocking Operation:** This is a blocking call. Returns when the robot reaches
     *       the target (within stop condition tolerance) or times out.
     *
     * @note **Waypoint Strategy:** For long distances, use multiple MoveTo() calls with
     *       intermediate waypoints. This reduces accumulated odometry error by periodically
     *       resetting the target.
     *
     * @warning **Odometry Accumulation:** Odometry errors accumulate over distance.
     *          For moves > 10 meters, significant drift is expected without correction.
     *          Periodically use absolute localization (vision, markers, GNSS) to correct position.
     *
     * @warning **Path Obstacles:** This method does not detect or avoid obstacles.
     *          Ensure a clear path to the target before calling. Collisions cause stalling.
     *
     * @warning **Unreachable Targets:** If the target is outside the robot's operating space
     *          or permanently blocked by obstacles, the robot will fail to reach it and may
     *          exhibit timeout or stalling behavior (depending on the stop condition).
     *
     * @see Move() for relative positioning
     * @see Orient() for heading-only rotation
     * @see BaseNavigation::MoveTo()
     */
    void MoveTo(float x, float y) override;

    /** 
     * @brief Orients the robot to a specific absolute angle in the global frame.
     * @details Rotates the robot so its heading (theta) matches the target absolute angle.
     *          Unlike Turn() which is relative, this is an absolute heading command.
     *          The robot takes the shortest rotation path to reach the target heading.
     *
     *          **Algorithm:**
     *          @code
     *          heading_error = target_angle - current_heading
     *          Normalize heading_error to [-π, π]  // Shortest path
     *          Turn(heading_error)                 // Execute relative turn
     *          @endcode
     *
     *          **Angle Normalization:**
     *          To always take the shortest rotation:
     *          - 170° to -170° = 20° error (shortest path)
     *          - -170° to 170° = 20° error (same as above)
     *          - 10° to 350° = -20° error = 20° in opposite direction
     *
     * @param angle The desired absolute heading in the global frame.
     *              - Theta = 0: Robot facing along positive X-axis (right)
     *              - Theta = π/2: Robot facing along positive Y-axis (forward)
     *              - Theta = π/-π: Robot facing along negative X-axis (left)
     *              - Theta = -π/2: Robot facing along negative Y-axis (backward)  
     *              Units: Radians
     *
     * @note **Absolute Heading:** This specifies an absolute target angle, not relative.
     *       Orient(0.0) sets heading to 0 regardless of current heading.
     *       Orient(π/2) sets heading to 90° regardless of current heading.
     *
     * @note **Shortest Path:** The implementation should automatically select the shortest
     *       rotation path (clockwise or counter-clockwise) to reach the target.
     *
     * @note **Blocking Operation:** This is a blocking call that returns when the rotation
     *       completes (stop condition satisfied).
     *
     * @note **Wraparound Handling:** Handles angle wraparound correctly:
     *       - Rotating from 10° to 350° takes the short path (20° backward)
     *       - Not the long path (340° forward)
     *
     * @warning **Gyro Drift:** Repeated Orient() calls over time accumulate gyro/odometry drift.
     *          For long-term missions, periodically correct absolute heading using external
     *          feedback (magnetometer, IMU, or external localization).
     *
     * @warning **Heading Singularities:** At heading = ±π, small implementation errors can
     *          cause wraparound issues (going from π to -π when tiny increments cross the boundary).
     *          Normalize to [-π, π) rather than (-π, π] to avoid edge cases.
     *
     * @see Turn() for relative rotation
     * @see MoveTo() for absolute position with rotation
     * @see BaseNavigation::Orient()
     */
    void Orient(float angle) override;

protected:
    /**
     * @brief Constructs a new DifferentialDriveNavigation object.
     * @details Protected constructor; use the Create() factory method instead.
     *          Initializes the navigation system with all required dependencies.
     *
     * @param odometryHandle Smart pointer to the differential drive odometry system.
     * @param profileGeneratorHandle Smart pointer to the motion profile generator.
     * @param stopConditionHandle Smart pointer to the stop condition.
     * @param motorConfig Structure containing motor and controller references.
     *
     * @post The navigation system is fully initialized and ready to accept motion commands.
     *
     * @note **Ownership:** All parameters remain owned by the caller. The navigation system
     *       holds references but doesn't own the objects. Cleanup is the caller's responsibility.
     */
    explicit DifferentialDriveNavigation(
        GenericDifferentialDriveOdometryHandle odometryHandle,
        BaseProfileGeneratorHandle profileGeneratorHandle,
        BaseStopConditionHandle stopConditionHandle,
        DifferentialDriveMotorConfig motorConfig);
};

} // namespace Motion::Core::Robot