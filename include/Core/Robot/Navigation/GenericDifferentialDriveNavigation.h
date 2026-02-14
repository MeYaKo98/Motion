/**
 * @file GenericDifferentialDriveNavigation.h
 * @brief Defines the navigation class for differential drive robots.
 */

#pragma once

#include "Core/Robot/Navigation/BaseNavigation.h"
#include "Core/Robot/Odom/GenericDifferentialDriveOdometry.h"
#include "Core/Robot/Controller/BaseController.h"
#include "Core/IO/Actuator/GenericMotor.h"

namespace Motion::Core::Robot {

/**
 * @brief Configuration structure for differential drive motors and controllers.
 * @details Holds pointers to the left and right motors and their respective controllers.
 *          Used to initialize the navigation system.
 */
struct DifferentialDriveMotorConfig {
    /** @brief Pointer to the right motor instance. */
    Motion::Core::IO::GenericMotor* rightMotor;
    /** @brief Pointer to the left motor instance. */
    Motion::Core::IO::GenericMotor* leftMotor;
    /** @brief Pointer to the controller for the right motor. */
    BaseController* rightController;
    /** @brief Pointer to the controller for the left motor. */
    BaseController* leftController;
};

/**
 * @brief A concrete navigation class for differential drive robots.
 * @details This class implements the navigation logic specific to differential drive kinematics.
 *          It utilizes an odometry system, a motion profile generator, and a stop condition
 *          to control the robot's movement via the configured motors and controllers.
 */
class GenericDifferentialDriveNavigation : public BaseNavigation{
public:
    /**
     * @brief Constructs a new GenericDifferentialDriveNavigation object.
     * @details Initializes the navigation system with the required components.
     *
     * @param odometry Pointer to the differential drive odometry instance.
     *                 Used for position tracking. Must not be nullptr.
     * @param profileGenerator Pointer to the profile generator (e.g., trapezoidal velocity profile).
     *                         Used to calculate target velocities. Must not be nullptr.
     * @param stopCondition Pointer to the stop condition instance.
     *                      Used to determine when the target is reached. Must not be nullptr.
     * @param motorConfig Structure containing pointers to the motors and controllers.
     *
     * @warning This class stores raw pointers to the dependencies. The caller must ensure
     *          that these objects remain valid for the lifetime of this navigation instance.
     */
    explicit GenericDifferentialDriveNavigation(GenericDifferentialDriveOdometry* odometry, BaseProfileGenerator* profileGenerator, BaseStopCondition* stopCondition, DifferentialDriveMotorConfig motorConfig)
        : _odometry(odometry), _motorConfig(motorConfig), BaseNavigation(profileGenerator, stopCondition) {
            if (_odometry == nullptr) LOG_ERROR("Odometry can not be NULL");
            if (_motorConfig.leftController == nullptr) LOG_ERROR("Left Controller can not be NULL");
            if (_motorConfig.leftMotor == nullptr) LOG_ERROR("Left Motor can not be NULL");
            if (_motorConfig.rightController == nullptr) LOG_ERROR("Right Controller can not be NULL");
            if (_motorConfig.rightMotor == nullptr) LOG_ERROR("Right Motor can not be NULL");
        }

    /**
     * @brief Destructor for the GenericDifferentialDriveNavigation object.
     */
    virtual ~GenericDifferentialDriveNavigation() = default;

protected:
    /**
     * @brief Pointer to the differential drive odometry system.
     */
    GenericDifferentialDriveOdometry* _odometry;

    /**
     * @brief Configuration object holding motor and controller references.
     */
    DifferentialDriveMotorConfig _motorConfig;
};

} // namespace Motion::Core::Robot