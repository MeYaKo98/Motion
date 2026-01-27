/**
 * @file DifferentialDriveOdometry.h
 * @brief Defines the DifferentialDriveOdometry class for tracking robot position.
 * @details This file contains the concrete implementation of odometry for a differential drive robot,
 *          calculating position and orientation based on wheel encoder feedback.
 */

#pragma once

#include "Core\Robot\Odom\GenericDifferentialDriveOdometry.h"

namespace Motion::Core::Robot {

/**
 * @brief Concrete implementation of odometry for a differential drive robot.
 * @details This class calculates the robot's position (x, y, theta) by monitoring the
 *          displacement of the left and right wheels. It inherits from `GenericDifferentialDriveOdometry`
 *          and implements the specific `OdometryUpdate` logic required for standard differential kinematics.
 * @see GenericDifferentialDriveOdometry
 * @see BaseOdometry
 */
class DifferentialDriveOdometry : public GenericDifferentialDriveOdometry {

public:
    /**
     * @brief Construct a new Differential Drive Odometry object.
     * @details Initializes the odometry system with the physical robot parameters and wheel references.
     * @param wheelSpacing The distance between the contact points of the left and right wheels (track width).
     *                     This value is critical for accurate turning angle calculations. Must be a positive value
     *                     in the same units as the wheel diameter.
     * @param rightWheel Pointer to the right wheel object used to read encoder data.
     * @param leftWheel Pointer to the left wheel object used to read encoder data.
     * @warning The `rightWheel` and `leftWheel` pointers must not be `nullptr`. Passing `nullptr` will result in undefined behavior.
     * @warning The caller is responsible for ensuring the `Wheel` objects remain valid for the lifetime of this odometry instance.
     */
    explicit DifferentialDriveOdometry(float wheelSpacing, Wheel* rightWheel, Wheel* leftWheel);

    /**
     * @brief Destroys the Differential Drive Odometry object.
     * @details Cleans up resources used by the odometry instance.
     * @note This destructor does **not** delete the `Wheel` objects passed in the constructor. 
     *       The ownership of `Wheel` objects remains with the caller.
     */
    ~DifferentialDriveOdometry();

private:
    /**
     * @brief Performs the periodic odometry calculation.
     * @details This function is called periodically by the background task (managed by `BaseOdometry`).
     *          It reads the current encoder values from the left and right wheels, calculates the
     *          incremental displacement and change in heading, and updates the robot's global position state.
     * @note The velocity calculation is updated at a lower frequency (decimated) compared to position integration
     *       to avoid quantization noise from the encoders at high sampling rates. The maximum velocity update frequency is 100Hz.
     * @note This method overrides the pure virtual function in `BaseOdometry`.
     */
    void OdometryUpdate() override;

    float _lastRightDistanceRef; ///< The last recorded distance of the right wheel, used to calculate velocity.
    float _lastLeftDistanceRef;  ///< The last recorded distance of the left wheel, used to calculate velocity.
    uint8_t _velCounter;         ///< Counter used to decimate velocity updates to reduce quantization noise.
};

} // namespace Motion::Core::Robot