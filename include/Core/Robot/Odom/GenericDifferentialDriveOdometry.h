/**
 * @file GenericDifferentialDriveOdometry.h
 * @brief Defines the generic differential drive odometry logic.
 */

#pragma once

#include "Core/Robot/Odom/BaseOdometry.h"
#include "Core/Robot/Wheel.h"

namespace Motion::Core::Robot {

/**
 * @brief A generic implementation of odometry for differential drive robots.
 * @details This class maintains the state of a differential drive robot (left and right wheel distances/speeds).
 *          It serves as a base for specific hardware implementations or as a logic handler for
 *          differential kinematics.
 */
class GenericDifferentialDriveOdometry : public BaseOdometry {
public:
    /**
     * @brief Represents the kinematic state of the differential drive wheels.
     */
    struct DifferentialDriveState {
        /** @brief Distance traveled by the right wheel. */
        float rightDistance;
        /** @brief Distance traveled by the left wheel. */
        float leftDistance;
        /** @brief Speed of the right wheel. */
        float rightEncoderSpeed;
        /** @brief Speed of the left wheel. */
        float leftEncoderSpeed;
    };

    /**
     * @brief Destroys the GenericDifferentialDriveOdometry object.
     */
    ~GenericDifferentialDriveOdometry();

    /**
     * @brief Retrieves the current state of the differential drive wheels.
     * @return DifferentialDriveState A struct containing distances and speeds for both wheels.
     */
    DifferentialDriveState GetState();

protected:
    /**
     * @brief Constructs a new GenericDifferentialDriveOdometry object.
     * @param wheelSpacing The distance between the centers of the two wheels (track width).
     * @param rightWheel Pointer to the right wheel object. Must not be nullptr.
     * @param leftWheel Pointer to the left wheel object. Must not be nullptr.
     * @warning The caller is responsible for ensuring the Wheel pointers remain valid for the lifetime of this object.
     */
    explicit GenericDifferentialDriveOdometry(float wheelSpacing, WheelHandle& rightWheelHandle, WheelHandle& leftWheelHandle);

    /**
     * @brief Set the current state of the differential drive wheels.
     * @param newState DifferentialDriveState A struct containing distances and speeds for both wheels.
     */
    bool SetState(const DifferentialDriveState& newstate);

    /** @brief The distance between wheels. */
    float _wheelSpacing;
    /** @brief Pointer to the right wheel instance. */
    WheelHandle _rightWheelHandle;
    /** @brief Pointer to the left wheel instance. */
    WheelHandle _leftWheelHandle;
  
private:
    /** 
     * @brief The current internal state of the drive. 
     */
    DifferentialDriveState _state;

    /**
     * @brief Protect access to the state (set and get).
     */
    SemaphoreHandle_t _stateMutex;
};

using GenericDifferentialDriveOdometryHandle = OdometryPointer(GenericDifferentialDriveOdometry);

} // namespace Motion::Core::Robot