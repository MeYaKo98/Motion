/**
 * @file BaseOdometry.h
 * @brief Defines the abstract base interface for robot odometry systems.
 * @details This file contains the BaseOdometry class, which manages the lifecycle of a FreeRTOS task
 *          dedicated to periodic position updates. It serves as a foundation for specific drive
 *          implementations (e.g., differential, omnidirectional, mecanum).
 *
 *          **Odometry Overview:**
 *          Odometry is the process of estimating a robot's position and orientation by integrating
 *          motion measurements from sensors (typically wheel encoders). As the robot moves, encoders
 *          count wheel rotations, which are converted to distances traveled, and these are integrated
 *          using kinematic equations to estimate the robot's pose (x, y, theta).
 *
 *          **Key Concepts:**
 *          - **Dead Reckoning:** Odometry calculates position based only on motion (no external reference)
 *          - **Accumulation Error:** Errors accumulate over distance (typically 1-5% per meter)
 *          - **Correction Needed:** Periodic recalibration with external reference (landmarks, GPS) is essential
 *          - **High Frequency:** Odometry updates must be frequent (100-1000 Hz) for accurate integration
 */

#pragma once

#include <stdint.h>
#include <memory>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "Core/Robot/Position.h"
#include "Core/Logger.h"

namespace Motion::Core::Robot {

#define OdometryPointer(T) std::shared_ptr<T>

/**
 * @brief An abstract base interface for odometry systems supporting all drive types.
 * @details This class provides the infrastructure for running a periodic background odometry task
 *          using FreeRTOS. The task executes the kinematic calculation at a configurable frequency,
 *          allowing real-time position tracking.
 *
 *          **Architecture:**
 *          - FreeRTOS Task Management: Creates and manages the background odometry task
 *          - Thread-Safe Position State: Position is protected by a mutex for concurrent access
 *          - Periodic Updates: OdometryUpdate() called at configured frequency (default 1000 Hz)
 *          - Abstract Kinematics: Derived classes implement drive-specific calculations
 *
 *          **Task Lifecycle:**
 *          1. Constructor: Initializes position to (0, 0, 0) and creates mutex
 *          2. Start(): Creates FreeRTOS task running OdometryTask()
 *          3. Running: Task calls OdometryUpdate() periodically
 *          4. Stop(): Deletes the FreeRTOS task and cleans up
 *          5. Destructor: Ensures task is stopped and mutex is deleted
 *
 *          **Typical Usage:**
 *          @code
 *          // Create odometry instance (derived class, e.g., DifferentialDriveOdometry)
 *          auto odometry = DifferentialDriveOdometry::Create(wheelSpacing, rightWheel, leftWheel);
 *          
 *          // Start background task at 100 Hz
 *          odometry->Start(100);
 *          
 *          // Read position in main loop
 *          Position pos = odometry->GetPosition(); // Thread-safe read
 *          
 *          // Correct odometry drift if needed
 *          Position corrected = {x_from_vision, y_from_vision, theta_from_imu};
 *          odometry->SetPosition(corrected);
 *          
 *          // Cleanup
 *          odometry->Stop();
 *          @endcode
 *
 * @note **FreeRTOS Requirement:** This implementation is tightly coupled to FreeRTOS.
 *       It will not compile or run on non-FreeRTOS systems without significant modification.
 *
 * @note **Frequency vs. Accuracy Trade-off:**
 *       - Higher frequency (500-1000 Hz): More accurate integration, higher CPU load
 *       - Lower frequency (50-100 Hz): Less CPU load, but integration error increases
 *       - Typical choice: 100-200 Hz for room-scale robots
 *
 * @see DifferentialDriveOdometry for a concrete two-wheel implementation
 * @see GenericDifferentialDriveOdometry for generic differential drive logic
 * @see Position for the pose structure used for state
 */
class BaseOdometry {
public:
    /**
     * @brief Virtual destructor for proper polymorphic cleanup.
     * @details Ensures the odometry task is stopped before destruction to prevent
     *          resource leaks or dangling task references. Cleans up the position mutex.
     *
     * @post The background odometry task is stopped (if running).
     * @post The position mutex is deleted if it exists.
     *
     * @note **Stop Called Automatically:** The destructor calls Stop(), ensuring
     *       proper cleanup even if the user forgets to call Stop() explicitly.
     *
     * @note **RAII Pattern:** This ensures the odometry system follows proper
     *       resource management: resources acquired in constructor, released in destructor.
     */
    virtual ~BaseOdometry();

    /**
     * @brief Starts the periodic background odometry update task.
     * @details Creates and starts a FreeRTOS task that calls the OdometryUpdate() method
     *          at the specified frequency, enabling continuous background position tracking.
     *
     *          **Task Creation Parameters:**
     *          - Task Name: "OdometryTask"
     *          - Stack Size: 2048 bytes
     *          - Priority: 10 (moderate priority, adjustable if needed)
     *          - Period: (1000 / odometryFrequency) milliseconds
     *          - Scheduling: Uses vTaskDelayUntil() for precise periodic execution
     *
     *          **Frequency Adjustment:**
     *          The Start() method enforces several constraints on the frequency:
     *          - Maximum: Clamped to 1000 Hz (limits per FreeRTOS 1ms tick resolution)
     *          - Divisor of 1000: Frequency must divide 1000 evenly
     *            If not, it's adjusted to nearest smaller divisor:
     *            e.g., 333 Hz → 250 Hz (auto-corrected)
     *
     * @param odometryFrequency The desired update frequency in Hertz (Hz).
     *                          Default: 1000 Hz.
     *                          Valid range: 1-1000 Hz.
     *                          Typical values: 50, 100, 200, 500, 1000.
     *
     * @pre The odometry system must be fully initialized. For DifferentialDriveOdometry,
     *      the wheels must be created and ready to provide readings.
     *
     * @pre Wheels must be started (wheel->Start()) before the odometry task begins,
     *      otherwise OdometryUpdate() will read invalid encoder data.
     *
     * @post The background task `OdometryTask()` is running and calling OdometryUpdate()
     *       periodically at the specified frequency.
     *
     * @post Position is now being continuously updated from sensor readings.
     *
     * @note **Idempotency:** If the task is already running, this method logs a warning
     *       and returns without creating a duplicate task. Call Stop() first to restart
     *       with a different frequency.
     *
     * @note **FreeRTOS Context:** This method must be called from a FreeRTOS task
     *       (i.e., not from an ISR or boot time before the scheduler starts).
     *
     * @note **Task Priority:** The task runs at priority 10. If your application has
     *       real-time requirements, you may need to adjust this in derived classes.
     *       Higher priority ensures more accurate periodic timing.
     *
     * @note **Default Frequency:** 1000 Hz means the timer ticks every 1 millisecond.
     *       This is the highest practical frequency on standard ESP32 FreeRTOS configurations.
     *
     * @note **Timing Precision:** Uses FreeRTOS's vTaskDelayUntil() for jitter-free
     *       periodic task triggering. Maintains consistent period even if OdometryUpdate()
     *       takes variable time (within limits).
     *
     * @warning **Sensor Initialization:** Ensure all sensors (encoders in wheels) are
     *          initialized and started before calling Start(). The first OdometryUpdate()
     *          call will read sensor values, so they must be ready.
     *
     * @warning **Already Running:** If the task is already running and you call Start()
     *          again with a different frequency, the new request is ignored. You must
     *          call Stop() first. Attempting to restart causes a warning log but no error.
     *
     * @warning **High Frequency Impact:** Frequencies above 500 Hz may cause excessive
     *          CPU load on resource-constrained systems, potentially starving other tasks.
     *          Monitor CPU usage and adjust if other subsystems become unresponsive.
     *
     * @warning **Low Frequency Trade-off:** Frequencies below 50 Hz significantly reduce
     *          odometry accuracy due to coarser integration of motion. Only use very low
     *          frequencies if CPU resources are critically limited.
     *
     * @see Stop()
     * @see OdometryUpdate()
     */
    void Start(uint16_t odometryFrequency = 1000);

    /**
     * @brief Retrieves the current estimated robot position.
     * @details Returns the robot's current pose (x, y, theta) estimated by the odometry system.
     *          The position is read in a thread-safe manner, so it can be called from any task.
     *
     *          **Return Value Components:**
     *          - x: Robot's X position in the global frame (typically meters)
     *          - y: Robot's Y position in the global frame (typically meters)
     *          - theta: Robot's heading/yaw in the global frame (radians)
     *
     * @return Position A Position struct containing {x, y, theta} at the time of the call.
     *                  This is a snapshot; the position changes as the robot moves.
     *
     * @note **Thread-Safety:** This method acquires a mutex to safely read the position
     *       from the background odometry task. Concurrent calls (from different tasks)
     *       are safe and will not corrupt data.
     *
     * @note **Snapshot Semantics:** The returned position is a snapshot at the call time.
     *       The actual robot position may have changed immediately after this call returns
     *       due to the background task running in parallel.
     *
     * @note **Timeout Handling:** If the mutex acquisition times out (5 ms), a warning is logged
     *       and the last known position is returned. Timeout indicates the odometry task
     *       is stuck or the system is severely overloaded.
     *
     * @note **Units:** Position coordinates use the same units as the wheel radius.
     *       - If wheels use meters (typical), position is in meters
     *       - If wheels use millimeters, position is in millimeters
     *
     * @note **Accumulation:** Position values accumulate continuously and can grow very large
     *       for long missions. For long-distance autonomy, periodically reset using SetPosition()
     *       with values from absolute localization (vision, GPS, etc.).
     *
     * @warning **Odometry Drift:** Without external correction, position drifts over time.
     *          Typical drift: 1-5% of distance traveled. For moves > 10 meters, expect
     *          significant error. Use periodic SetPosition() calls with corrected values.
     *
     * @warning **Stale Data on Timeout:** If the mutex times out, the returned position may be
     *          several milliseconds old (from the last successful read). Check return value of
     *          SetPosition() for indication of locking problems.
     *
     * @see SetPosition()
     * @see Position
     */
    Position GetPosition();

    /**
     * @brief Sets the robot position to a specific value.
     * @details Allows manual correction of the odometry position. Useful for:
     *          - Resetting to a known starting position at initialization
     *          - Correcting accumulated drift using external localization (vision, GNSS, landmarks)
     *          - Teleporting the robot's internal position for simulation or testing
     *
     *          **Typical Usage Pattern:**
     *          @code
     *          // Robot has traveled some distance but odometry accumulated error
     *          // Use vision system to determine corrected position
     *          Position correctedPos = visionSystem->GetRobotPose();
     *          if (odometry->SetPosition(correctedPos))
     *          {
     *              LOG_TRACE("Odometry corrected");
     *          } else
     *          {
     *              LOG_WARN("Warning: odometry correction failed (mutex timeout)");
     *          }
     *          @endcode
     *
     *          **Loop Closure Example:**
     *          @code
     *          // Robot returns to starting location; vision confirms it
     *          Position homeOnVision = {0.0f, 0.0f, 0.0f};
     *          Position homeOnOdometry = odometry->GetPosition();
     *          // Drift detected; correct odometry
     *          odometry->SetPosition(homeOnVision);
     *          LOG_INFO("Loop closure: odometry corrected by [%.3f, %.3f, %.3f]m",
     *              homeOnOdometry.x - homeOnVision.x,
     *              homeOnOdometry.y - homeOnVision.y,
     *              homeOnOdometry.theta - homeOnVision.theta);
     *          @endcode
     *
     * @param newPosition The new position to set {x, y, theta}.
     *                    - x, y: Linear position in the global frame (application units, typically meters)
     *                    - theta: Heading in the global frame (radians)
     *
     * @return bool true if the position was successfully set.
     *         false if the operation failed (e.g., mutex timeout after 5 ms wait).
     *
     * @post If true: The internal position state is updated to newPosition. Subsequent
     *       calls to GetPosition() will return (approximately) this new value.
     *
     * @post If false: The position is unchanged. The set operation was not performed.
     *
     * @note **Thread-Safety:** This method acquires the same mutex as GetPosition().
     *       It is safe to call from any task, including the main control loop.
     *
     * @note **Atomic Update:** The mutex ensures the entire position struct is updated
     *       atomically. There is no risk of reading a partially-updated position.
     *
     * @note **Timeout Handling:** Mutex acquisition times out after 5 ms. If the odometry
     *       task is holding the mutex, the call will retry for 5 ms before giving up.
     *       This prevents deadlock but may indicate system overload.
     *
     * @note **Typical Correction Interval:** For moving robots, correction every 5-10 seconds
     *       (or every 1-5 meters of travel) is typical. Too-frequent correction suppresses
     *       the odometry feedback; too-infrequent correction allows drift to accumulate.
     *
     * @warning **Discontinuity:** Setting position creates a discontinuity in the estimated
     *          path. Derivatives (velocity) computed from successive positions will show a spike.
     *          Filter or smooth the position history if derivatives are used downstream.
     *
     * @warning **Teleportation Risk:** Overuse of SetPosition() can hide actual hardware problems.
     *          If odometry constantly needs correction, investigate the root cause (wheel slip,
     *          encoder calibration, surface properties) rather than just correcting the error.
     *
     * @warning **Mutex Timeout:** If this method returns false (timeout), it indicates:
     *          - The odometry task is stuck in OdometryUpdate()
     *          - System is severely overloaded
     *          - Potential deadlock or priority inversion
     *          Investigate and fix the underlying cause.
     *
     * @see GetPosition()
     * @see Position
     */
    bool SetPosition(const Position& newPosition);

    /**
     * @brief Stops the background odometry update task.
     * @details Gracefully shuts down the periodic odometry calculation by deleting the FreeRTOS task,
     *          releasing all associated resources. After calling this, the odometry system is dormant.
     *          Position readings will return the last cached value but will no longer be updated.
     *
     *          **Cleanup Actions:**
     *          1. If task is running: Calls vTaskDelete() to terminate it
     *          2. Resets _odomTaskHandler to nullptr
     *          3. Clears _started flag
     *          4. Logs "Odometry stopped" message
     *
     *          **Resource Impact:**
     *          - Frees ~2048 bytes of stack memory used by the task
     *          - Releases the task's CPU time budget back to the scheduler
     *          - Stops sensor polling (encoders are no longer read periodically)
     *
     * @post The background odometry task is stopped and its resources are freed.
     * @post _odomTaskHandler is set to nullptr.
     * @post _started flag is cleared (false).
     * @post Position is no longer being updated. GetPosition() returns stale data.
     *
     * @note **Idempotency:** Calling Stop() when the task is not running is safe—it simply
     *       returns without error. Multiple calls to Stop() do nothing after the first.
     *
     * @note **Task Termination:** FreeRTOS's vTaskDelete() is a blocking operation that waits
     *       for the task to be fully deleted. This may take a few context switches (milliseconds).
     *       Stop() does not return until the task is confirmed deleted.
     *
     * @note **Cleanup in Destructor:** The destructor calls Stop() automatically,
     *       so explicitly calling Stop() is optional (but recommended for clarity).
     *
     * @warning **Position Corruption Risk:** The mutex protecting position is NOT deleted
     *          when Stop() is called. If OdometryUpdate() is hanging (stuck on mutex), Stop()
     *          will not complete immediately. Investigate if Stop() appears to hang.
     *
     * @warning **Residual Reads:** After Stop(), GetPosition() will still return the last
     *          cached position (no longer updated). Code that depends on fresh position data
     *          must check whether odometry is running before using the value.
     *
     * @see Start()
     * @see GetPosition()
     */
    void Stop();

protected:
    /**
     * @brief Constructs a new BaseOdometry object.
     * @details Initializes all internal state variables and creates the position mutex.
     *          The odometry task is NOT started until Start() is called.
     *
     *          **Initialization:**
     *          - _position: Set to (0, 0, 0)
     *          - _positionMutex: Created (binary semaphore used as mutex)
     *          - _started: Set to false
     *          - _odometryFrequency: Set to 1000 Hz
     *          - _odomTaskHandler: Set to nullptr
     *
     * @post All state is initialized and ready for use.
     * @post The position mutex is valid and ready for synchronization.
     *
     * @throws std::runtime_error if mutex creation fails (critical FreeRTOS failure).
     *
     * @note **Thread-Safe Construction:** The constructor is not itself thread-safe
     *       (this is typical for C++ constructors), but it initializes mutexes that
     *       enable thread-safe operation afterward.
     *
     * @note **Must Not Be Called Directly:** This is protected; only derived classes
     *       call it via super(). Use the derived class's Create() factory instead.
     */
    BaseOdometry();

    /**
     * @brief The configured odometry update frequency in Hertz.
     * @details Stored for reference and used to configure the periodic task.
     * @note Read/modified by Start(); should not be changed directly.
     * @note Range: 1-1000 Hz. Typically 100-500 Hz.
     */
    int16_t _odometryFrequency;

    /**
     * @brief Flag indicating whether the odometry task is currently running.
     * @details Set to true by Start(), false by Stop().
     * @note Used to prevent starting duplicate tasks.
     */
    bool _started;

private:
    /**
     * @brief Handle to the FreeRTOS odometry task.
     * @details Holds the task ID. nullptr when task is not running.
     * @note Used by Stop() to delete the task.
     */
    TaskHandle_t _odomTaskHandler;

    /**
     * @brief The static FreeRTOS task function entry point.
     * @details This static function is called by FreeRTOS when the task starts.
     *          It loops periodically, calling OdometryUpdate() at the configured frequency.
     *
     *          **Task Loop:**
     *          @code
     *          void OdometryTask(void* pvParameters)
     *          {
     *              BaseOdometry* odom = static_cast<BaseOdometry*>(pvParameters);
     *              TickType_t period = pdMS_TO_TICKS(1000 / frequency);
     *              TickType_t lastWakeTime = xTaskGetTickCount();
     *              while (true)
     *              {
     *                  odom->OdometryUpdate();  // Call derived implementation
     *                  vTaskDelayUntil(&lastWakeTime, period);  // Precise periodic delay
     *              }
     *          }
     *          @endcode
     *
     * @param pvParameters Pointer to the BaseOdometry instance (cast from void* to BaseOdometry*)
     *
     * @note **Static Function:** Required by FreeRTOS task API. Allows calling non-static
     *       member functions through the "this" pointer embedded in pvParameters.
     *
     * @note **Infinite Loop:** The task runs forever until vTaskDelete() is called.
     *       Periodic delays are achieved with vTaskDelayUntil(), which provides jitter-free timing.
     *
     * @see Start() for task creation and parameter passing
     */
    static void OdometryTask(void* pvParameters);

    /**
     * @brief Updates odometry data based on current sensor readings.
     * @details This pure virtual method must be implemented by derived classes to perform
     *          drive-specific kinematic calculations. It is called periodically by the OdometryTask
     *          at the configured frequency (e.g., 100-1000 Hz).
     *
     *          **Implementation Examples:**
     *          - **DifferentialDriveOdometry:** Reads left/right wheel distances, computes position
     *            change using differential drive kinematics, updates _position.
     *          - **OmniDirectionalOdometry:** Reads three wheel distances, computes position
     *            using holonomic kinematics.
     *          - **IMU-Based:** Reads accelerometer/gyro, integrates for position/orientation.
     *
     *          **Algorithm Template:**
     *          @code
     *          void OdometryUpdate() override
     *          {
     *              // 1. Read sensor data
     *              float leftDist = leftWheel->getDistance();
     *              float rightDist = rightWheel->getDistance();
     *              
     *              // 2. Compute deltas
     *              float deltaLeft = leftDist - _lastLeftDist;
     *              float deltaRight = rightDist - _lastRightDist;
     *              
     *              // 3. Apply kinematics
     *              float avgDelta = (deltaLeft + deltaRight) / 2;
     *              float deltaThetaRad = (deltaRight - deltaLeft) / wheelSpacing;
     *              
     *              // 4. Update position
     *              Position currPos = GetPosition();
     *              Position newPos;
     *              newPos.x = currPos.x + avgDelta * cos(currPos.theta);
     *              newPos.y = currPos.y + avgDelta * sin(currPos.theta);
     *              newPos.theta = currPos.theta + deltaThetaRad;
     *              
     *              SetPosition(newPos);
     *          }
     *          @endcode
     *
     * @note **Called Periodically:** This method is invoked by OdometryTask() at the configured
     *       frequency. Default 1000 Hz means it's called 1000 times per second (~1 ms interval).
     *
     * @note **Real-Time Constraints:** Keep execution time short (ideally < 1 ms) to avoid
     *       causing task timing jitter or starving other tasks. Avoid I/O, dynamic allocation,
     *       or long calculations.
     *
     * @note **Sensor Error Handling:** Handle sensor failures gracefully (e.g., encoder read fails).
     *       Return last position or use dead-reckoning fallback rather than crashing.
     *
     * @note **Velocity Decimation:** For better velocity accuracy, many implementations skip
     *       velocity calculation on every call and only compute it every 10th call (decimation).
     *       This reduces quantization noise from discrete encoder counts.
     *
     * @warning **Thread Safety:** This method is called from the odometry task but may run
     *          concurrently with calls to GetPosition() / SetPosition() which acquire the mutex.
     *          Use SetPosition() internally or acquire _positionMutex when updating _position.
     *
     * @warning **Sensor Readiness:** Assumes all sensors are started and ready.
     *          If a sensor is not started, reading it may return stale/zero data or crash.
     *          Verify sensor initialization before calling Start().
     *
     * @see BaseOdometry for the framework
     * @see DifferentialDriveOdometry for a concrete example
     */
    virtual void OdometryUpdate() = 0;

    /**
     * @brief The current estimated position of the robot in the global frame.
     * @details Contains x, y linear position and theta angular orientation.
     * @note Protected by _positionMutex for thread-safe concurrent access.
     * @note Initialized to (0, 0, 0) in constructor.
     * @note Updated by OdometryUpdate() and SetPosition().
     */
    Position _position;

    /**
     * @brief Protects concurrent access to the position state.
     * @details A FreeRTOS binary semaphore (mutex) ensuring thread-safe reads/writes of _position.
     * @note Acquired by GetPosition() and SetPosition().
     * @note Also acquired by OdometryUpdate() implementations that modify _position.
     * @note Timeout: 5 ms. If not acquired in 5 ms, operation fails.
     */
    SemaphoreHandle_t _positionMutex;
};

using BaseOdometryHandle = OdometryPointer(BaseOdometry);

} // namespace Motion::Core::Robot