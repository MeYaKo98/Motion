/**
 * @file BaseOdometry.h
 * @brief Defines the abstract base interface for robot odometry systems.
 * @details This file contains the BaseOdometry class, which manages the lifecycle of a FreeRTOS task
 *          dedicated to periodic position updates. It serves as a foundation for specific drive
 *          implementations (e.g., differential, omnidirectional).
 */

#pragma once

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "Core/Robot/Util.h"
#include "Core/Diagnostics/Logger.h"

namespace Motion::Core::Robot {

/**
 * @brief An abstract interface for odometry for all drive types.
 * @details This class provides the infrastructure for running a periodic odometry update task
 *          using FreeRTOS. Concrete implementations must define the `OdometryUpdate` method
 *          to perform the specific kinematic calculations for the drive type (e.g., differential, mecanum).
 */
class BaseOdometry {
public:
    /**
     * @brief Construct a new Base Odometry object.
     * @details Initializes internal state variables. The odometry task is not started until `Start()` is called.
     */
    BaseOdometry();

    /**
     * @brief Destructor of the Base Odometry object.
     * @details Ensures the odometry task is stopped before destruction to prevent resource leaks or dangling task references.
     */
    virtual ~BaseOdometry();

    /**
     * @brief Starts the periodic odometry update task.
     * @details Creates and starts a FreeRTOS task that executes the `OdometryUpdate` method at the specified frequency.
     *          This enables continuous background tracking of the robot's position.
     * @param odometryFrequency The frequency in Hertz (Hz) for the update loop. Default is 1000 Hz.
     *                          Must be a positive value. For accurate timing on standard FreeRTOS configurations (1ms tick),
     *                          this value should be a divisor of 1000 (e.g., 50, 100, 200, 500, 1000).
     * @note If the task is already running, this function does nothing. You must call `Stop()` first to restart with a new frequency.
     * @warning Ensure that the derived class is fully initialized before calling this method.
     *          Starting the task before sensors are ready may lead to undefined behavior or crashes.
     */
    void Start(uint16_t odometryFrequency = 1000);

    /**
     * @brief Get the current robot position.
     * @return Position The current estimated position (x, y, theta) of the robot.
     * @note This method is thread safe.
     */
    Position GetPosition();

    /**
     * @brief Set the robot position to a specific value.
     * @details Useful for resetting the odometry to a known starting point or correcting drift.
     * @param newPosition The new position to set.
     * @return true if the new position was successfully set.
     * @return false if the operation failed (e.g., mutex timeout).
     * @note This method is thread safe
     */
    bool SetPosition(const Position& newPosition);

    /**
     * @brief Stop and kill the odometry task.
     * @details Deletes the FreeRTOS task if it is running, resets the started flag, and cleans up task resources.
     *          This method blocks until the task is confirmed deleted or a timeout occurs.
     */
    void Stop();

private:
    /**
     * @brief The FreeRTOS task function.
     * @details This static function serves as the entry point for the FreeRTOS task. It casts the
     *          parameter back to a `BaseOdometry` instance and executes the update loop at the configured frequency.
     * @param pvParameters A pointer to the `BaseOdometry` instance (this).
     */
    static void OdometryTask(void* pvParameters);

    /**
     * @brief Update Odometry data.
     * @details Pure virtual function to be implemented by derived classes. This function is called
     *          periodically by the `OdometryTask` at the frequency specified in `Start()`.
     *          Implementations should read sensors (encoders, IMU) and update the `_position` member.
     */
    virtual void OdometryUpdate() = 0;

protected:
    /**
     * @brief The update frequency in Hz.
     */
    int16_t _odometryFrequency;

    /**
     * @brief Flag indicating if the odometry task is currently running.
     */
    bool _started;

    /**
     * @brief Handle to the FreeRTOS odometry task.
     */
    TaskHandle_t _odomTaskHandler;

private:
    /**
     * @brief The current calculated position of the robot.
     * @note Derived classes should update this member within `OdometryUpdate`.
     */
    Position _position;

    /**
     * @brief Protect access to the position (set and get).
     */
    SemaphoreHandle_t _positionMutex;
};

} // namespace Motion::Core::Robot