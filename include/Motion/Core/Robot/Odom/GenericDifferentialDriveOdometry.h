/**
 * @file GenericDifferentialDriveOdometry.h
 * @brief Defines the generic differential drive odometry system.
 * @details Provides common state management and kinematic calculations for differential drive robots.
 */

#pragma once

#include "Motion/Core/Robot/Odom/BaseOdometry.h"
#include "Motion/Core/Robot/Wheel.h"

namespace Motion::Core::Robot {

/**
 * @brief A generic implementation of odometry for differential drive robots.
 * @details This class maintains the state of a differential drive robot (left and right wheel
 *          distances and speeds) and handles common odometry operations. It serves as a
 *          foundation for specific hardware implementations and provides the common logic
 *          for differential kinematics calculations.
 *
 *          **Differential Drive Kinematics:**
 *          A differential drive robot has two independently-controlled wheels. By varying the
 *          speeds of the two wheels, the robot can:
 *          - Move straight (both wheels same speed)
 *          - Turn (wheels at different speeds)
 *          - Spin in place (wheels at opposite speeds)
 *
 *          **State Tracking:**
 *          The odometry regularly reads encoder values from both wheels and calculates:
 *          - Individual wheel distances traveled
 *          - Individual wheel speeds (velocity)
 *          - Global robot position (x, y, theta) using kinematic equations
 *
 * @note **Inherits from BaseOdometry:** This class builds on BaseOdometry, which handles
 *       the FreeRTOS task management and periodic position updates.
 *
 * @note **Thread-Safe State:** Position and state are protected by mutexes (inherited from BaseOdometry)
 *       to allow safe concurrent access from multiple tasks.
 *
 * @see BaseOdometry for the base class with task management
 * @see DifferentialDriveOdometry for a concrete hardware-specific implementation
 * @see Wheel for the individual wheel abstraction
 */
class GenericDifferentialDriveOdometry : public BaseOdometry {
public:
    /**
     * @struct DifferentialDriveState
     * @brief Represents the kinematic state of the differential drive wheels.
     * @details Holds measurements for both wheels at a specific point in time.
     *          Used to track and update the odometry state safely.
     */
    struct DifferentialDriveState {
        /** @brief Distance traveled by the right wheel (meters or application-specific units). */
        float rightDistance;

        /** @brief Distance traveled by the left wheel (meters or application-specific units). */
        float leftDistance;

        /** @brief Speed of the right wheel (m/s or application-specific velocity units).
         *  Represents the instantaneous or recent-average wheel rotation speed.
         */
        float rightEncoderSpeed;

        /** @brief Speed of the left wheel (m/s or application-specific velocity units).
         *  Represents the instantaneous or recent-average wheel rotation speed.
         */
        float leftEncoderSpeed;
    };

    /**
     * @brief Destroys the GenericDifferentialDriveOdometry object.
     * @details Cleans up any allocated resources, including the FreeRTOS task
     *          and synchronization primitives inherited from BaseOdometry.
     */
    ~GenericDifferentialDriveOdometry();

    /**
     * @brief Retrieves the current state of the differential drive wheels.
     * @details Reads and returns the current wheel distances and speeds in a thread-safe manner.
     *          The returned state represents a snapshot of the odometry at the time of the call.
     *
     * @return DifferentialDriveState A struct containing:
     *         - rightDistance: Total distance traveled by right wheel
     *         - leftDistance: Total distance traveled by left wheel
     *         - rightEncoderSpeed: Current right wheel speed
     *         - leftEncoderSpeed: Current left wheel speed
     *
     * @note **Thread-Safety:** This method acquires a mutex to read state safely.
     *       It blocks briefly (millisecond scale) if another task is currently writing state.
     *
     * @note **Snapshot:** The returned state is a snapshot at the call time.
     *       The state may change immediately after this method returns due to the
     *       background odometry update task running concurrently.
     *
     * @note **Units:** The returned distances and speeds use the same units as the wheels
     *       (determined by wheel radius and encoder resolution).
     *
     * @warning **Blocking Operation:** This method acquires a mutex and may block briefly.
     *          Avoid calling from interrupt handlers or time-critical control loops.
     *          Call from normal FreeRTOS tasks instead.
     *
     * @see SetState()
     * @see BaseOdometry::GetPosition()
     */
    DifferentialDriveState GetState();

protected:
    /**
     * @brief Constructs a new GenericDifferentialDriveOdometry object.
     * @details Protected constructor; typically called by derived classes through super() calls.
     *          Initializes the odometry system with references to both wheels and the wheel spacing.
     *
     * @param wheelSpacing The distance between the centers of the two wheels (track width).
     *                     Critical for accurate turning angle calculations.
     *                     Units must match the wheel radius units (typically meters).
     *                     Typical values: 0.1 - 1.0 meters for robots.
     *
     * @param rightWheelHandle Reference to the right Wheel instance. Must not be nullptr.
     * @param leftWheelHandle Reference to the left Wheel instance. Must not be nullptr.
     *
     * @post The odometry system is initialized with the provided wheel configuration.
     *       The background task is NOT started yet; call Start() to begin.
     *
     * @warning **Wheel Validity:** Ensure the wheel handles point to valid, initialized Wheel objects.
     *          The odometry will read from these wheels during operation.
     *
     * @warning **Lifetime:** The wheel objects must remain valid for the entire lifetime
     *          of the odometry instance. If wheels are destroyed, reading from them causes
     *          undefined behavior (likely a crash).
     */
    explicit GenericDifferentialDriveOdometry(float wheelSpacing, WheelHandle& rightWheelHandle, WheelHandle& leftWheelHandle);

    /**
     * @brief Sets the current state of the differential drive wheels.
     * @details Updates the internal wheel state (distances and speeds) in a thread-safe manner.
     *          This is typically called by the OdometryUpdate() method after the
     *          kinematic calculations.
     *
     * @param newState DifferentialDriveState A struct containing new distances and speeds
     *                 to update the internal state. All fields must be valid.
     *
     * @return true if the state was successfully updated.
     * @return false if the update failed (e.g., mutex timeout after waiting).
     *
     * @post If successful, `GetState()` will return the updated state on the next call.
     *
     * @note **Thread-Safety:** Acquires a mutex to update state. Brief blocking may occur.
     *
     * @note **Failure Recovery:** If this method returns false (timeout), the old state is preserved.
     *       Retry or log an error as appropriate for your application.
     *
     * @warning **Timeout Risk:** Setting state requires acquiring a mutex. If the mutex is
     *          already held (e.g., by a GetState() call in another task), this will block.
     *          Extended blocking may cause dropped odometry updates.
     *
     * @see GetState()
     */
    bool SetState(const DifferentialDriveState& newState);

    /** 
     * @brief The distance between the contact points of the two wheels (track width).
     * @details Critical parameter for kinematic calculations. Used to convert differential
     *          wheel speeds to robot heading rate: dtheta/dt = (v_right - v_left) / wheelSpacing
     * @note Units must match wheel radius (typically meters).
     * @note This value is constant; set during construction.
     */
    float _wheelSpacing;

    /** 
     * @brief Handle to the right wheel instance.
     * @details The odometry reads distance and encoder readings from this wheel.
     * @note Must not be nullptr; invalid reference causes crashes during OdometryUpdate().
     */
    WheelHandle _rightWheelHandle;

    /** 
     * @brief Handle to the left wheel instance.
     * @details The odometry reads distance and encoder readings from this wheel.
     * @note Must not be nullptr; invalid reference causes crashes during OdometryUpdate().
     */
    WheelHandle _leftWheelHandle;
  
private:
    /** 
     * @brief The current internal state of the differential drive wheels.
     * @details Holds the latest distance and speed measurements for both wheels.
     *          Updated periodically by the OdometryUpdate() task.
     *          Shared between the odometry task and caller tasks through _stateMutex.
     */
    DifferentialDriveState _state;

    /**
     * @brief Mutex protecting access to the state structure.
     * @details Ensures thread-safe read/write of _state when called from multiple tasks.
     *          Get/SetState() methods acquire this mutex before accessing _state.
     * @note FreeRTOS SemaphoreHandle_t (implements a mutex using a binary semaphore).
     */
    SemaphoreHandle_t _stateMutex;
};

using GenericDifferentialDriveOdometryHandle = OdometryPointer(GenericDifferentialDriveOdometry);

} // namespace Motion::Core::Robot