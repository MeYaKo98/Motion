/**
 * @file GenericDifferentialDriveOdometry.h
 * @brief Defines the generic differential drive odometry logic.
 */

#pragma once

#include "Core\Robot\Odom\BaseOdometry.h"
#include "Core\Robot\Wheel.h"

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
     * @brief Constructs a new GenericDifferentialDriveOdometry object.
     * @param wheelSpacing The distance between the centers of the two wheels (track width).
     * @param rightWheel Pointer to the right wheel object. Must not be nullptr.
     * @param leftWheel Pointer to the left wheel object. Must not be nullptr.
     * @warning The caller is responsible for ensuring the Wheel pointers remain valid for the lifetime of this object.
     */
    explicit GenericDifferentialDriveOdometry(float wheelSpacing, Wheel* rightWheel, Wheel* leftWheel) :
        _wheelSpacing(wheelSpacing), _rightWheel(rightWheel), _leftWheel(leftWheel), _state({0.0f, 0.0f, 0.0f, 0.0f}) {}

    /**
     * @brief Destroys the GenericDifferentialDriveOdometry object.
     */
    ~GenericDifferentialDriveOdometry() = default;

    /**
     * @brief Retrieves the current state of the differential drive wheels.
     * @return DifferentialDriveState A struct containing distances and speeds for both wheels.
     */
    DifferentialDriveState GetState() {return _state;}

private:

protected:
    /** @brief The distance between wheels. */
    float _wheelSpacing;
    /** @brief Pointer to the right wheel instance. */
    Wheel* _rightWheel;
    /** @brief Pointer to the left wheel instance. */
    Wheel* _leftWheel;
  
    /** @brief The current internal state of the drive. 
     * @todo add thread safety
     */
    DifferentialDriveState _state;
};

} // namespace Motion::Core::Robot