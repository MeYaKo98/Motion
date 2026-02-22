/**
 * @file DifferentialDriveOdometry.h
 * @brief Concrete differential drive odometry implementation for tracking robot position.
 * @details Implements odometry calculations specific to differential drive robots using
 *          wheel encoder feedback. Inherits from GenericDifferentialDriveOdometry to leverage
 *          common differential kinematics logic.
 */

#pragma once

#include "Core/Robot/Odom/GenericDifferentialDriveOdometry.h"

namespace Motion::Core::Robot {

/**
 * @brief Concrete implementation of odometry for a differential drive robot.
 * @details This class calculates the robot's 2D position (x, y, theta) by monitoring the
 *          displacement of the left and right drive wheels. It inherits from GenericDifferentialDriveOdometry,
 *          which provides common state management and differential kinematics utilities.
 *
 *          **Differential Drive Kinematics:**
 *          A differential drive robot has two independently-controlled wheels. The robot's motion
 *          is fully determined by the speeds/distances of these two wheels:
 *
 *          Given:
 *          - L: distance between wheels (track width), meters
 *          - v_l: velocity of left wheel, m/s
 *          - v_r: velocity of right wheel, m/s
 *          - d_l: incremental distance traveled by left wheel, meters
 *          - d_r: incremental distance traveled by right wheel, meters
 *
 *          Calculated:
 *          - d_forward = (d_r + d_l) / 2  (average distance)
 *          - dtheta = (d_r - d_l) / L      (heading change in radians)
 *          - Position update: x += d_forward * cos(theta), y += d_forward * sin(theta), theta += dtheta
 *
 *          **Typical Accuracy:**
 *          - 1-3% error per meter traveled (high-quality encoders on smooth floor)
 *          - 3-5% error per meter (consumer-grade encoders or uneven surface)
 *          - Errors accumulate; correction needed for long distances (> 10 meters)
 *
 *          **Assumptions:**
 *          - Both wheels have the same radius (wheel_radius)
 *          - Wheels roll without slipping
 *          - Track width is accurately measured and constant
 *          - Encoder resolution is accurate and consistent
 *
 * @note **Integration Strategy:** Position is integrated from incremental wheel displacements.
 *       Fast, efficient, but errors accumulate. Best for short-term positioning (seconds to minutes).
 *
 * @note **Coordinate Frame:** Uses standard robotics convention:
 *       - X-axis: to the right
 *       - Y-axis: forward
 *       - Theta: counter-clockwise from X-axis (right-hand rule)
 *       - Theta = 0: heading right, Theta = π/2: heading forward
 *
 * @note **Velocity Calculation:** Odometry computes velocity in addition to position.
 *       Velocity is decimated (computed less frequently than position) to reduce quantization noise.
 *
 * @see BaseOdometry for the periodic update framework
 * @see GenericDifferentialDriveOdometry for shared differential drive logic
 * @see Wheel for the wheel encoder abstraction
 * @see Position for the pose structure
 */
class DifferentialDriveOdometry;

using DifferentialDriveOdometryHandle = OdometryPointer(DifferentialDriveOdometry);

class DifferentialDriveOdometry : public GenericDifferentialDriveOdometry {

public:

    /**
     * @brief Factory method to create a DifferentialDriveOdometry instance.
     * @details Creates a new odometry system with full validation of wheel references and geometry.
     *
     * @param wheelSpacing The distance between the contact points of the left and right drive wheels (track width).
     *                     Critical for accurate turning angle calculations via: dtheta = (d_right - d_left) / wheelSpacing
     *                     Units: Same as wheel radius (typically meters).
     *                     Must be positive. Typical range: 0.1-1.0 meters.
     *                     Measure carefully; errors here cause systematic rotation drift.
     *
     * @param rightWheelHandle Smart pointer to the right Wheel instance. Must not be nullptr.
     *                         The wheel should be initialized but does not need to be started yet
     *                         (Start() will be called on the wheel when odometry starts).
     *
     * @param leftWheelHandle Smart pointer to the left Wheel instance. Must not be nullptr.
     *                        The wheel should be initialized but does not need to be started yet.
     *
     * @return DifferentialDriveOdometryHandle A shared pointer to the new odometry instance.
     *
     * @throws std::invalid_argument if wheelSpacing <= 0
     * @throws std::invalid_argument if either wheel handle is nullptr
     *
     * @note **Typical Usage:**
     *       @code
     *       // Create wheels with encoders
     *       auto rightEncoder = ESP32Encoder::Create("Right Encoder", {GPIO_A, GPIO_B});
     *       auto leftEncoder = ESP32Encoder::Create("Left Encoder", {GPIO_C, GPIO_D});
     *       auto rightWheel = Wheel::Create(rightEncoder, 64, 0.05f); // 64 PPR, 5cm radius
     *       auto leftWheel = Wheel::Create(leftEncoder, 64, 0.05f);
     *       
     *       // Create odometry
     *       auto odometry = DifferentialDriveOdometry::Create(0.15f, rightWheel, leftWheel);
     *       
     *       // Start wheels and odometry
     *       rightWheel->Start();
     *       leftWheel->Start();
     *       odometry->Start(100); // 100 Hz updates
     *       @endcode
     *
     * @warning **Wheel Lifetime:** The wheels passed to this factory must remain valid for
     *          the entire lifetime of the odometry instance. If wheels are deleted externally,
     *          accessing them causes undefined behavior (likely crash).
     *
     * @see Wheel for wheel creation
     * @see Wheel::Create()
     */
    static DifferentialDriveOdometryHandle Create(float wheelSpacing, WheelHandle& rightWheelHandle, WheelHandle& leftWheelHandle);

    /**
     * @brief Destroys the DifferentialDriveOdometry object.
     * @details Cleans up resources used by the odometry instance.
     *          The destructor calls the parent destructor, which stops the background task.
     *
     * @post The background odometry task is stopped (if running).
     * @post Resources are cleaned up.
     *
     * @note **Wheel Lifetime:** This destructor does NOT delete the Wheel objects passed in the constructor.
     *       The wheels remain the responsibility of their original creator/owner. Using smart pointers
     *       for wheels ensures they remain valid as long as any component references them.
     *
     * @note **RAII Pattern:** Follows proper resource management: constructor acquires resources,
     *       destructor releases them.
     */
    ~DifferentialDriveOdometry();

protected:
    /**
     * @brief Constructs a new DifferentialDriveOdometry instance.
     * @details Protected constructor; use the Create() factory method instead.
     *          Initializes the odometry system with physical robot parameters and wheel references.
     *
     * @param wheelSpacing The distance between the wheels' contact points (track width) typically in meters.
     *                     Critical parameter for accurate kinematics. Check measurement with tape measure.
     *
     * @param rightWheelHandle Smart pointer to the right Wheel. Used to read distance/speed via wheel->getDistance().
     * @param leftWheelHandle Smart pointer to the left Wheel. Used to read distance/speed.
     *
     * @post All state is initialized. The system is ready for Start().
     *
     * @note **Immutable Geometry:** Wheel spacing is constant after construction.
     *       If wheel spacing needs to be adjusted, create a new odometry instance.
     */
    explicit DifferentialDriveOdometry(float wheelSpacing, WheelHandle& rightWheelHandle, WheelHandle& leftWheelHandle);

    /**
     * @brief Performs periodic odometry calculation using differential drive kinematics.
     * @details Executed periodically by the background task (created by Start()).
     *          Reads current encoder values from both wheels, computes incremental displacement,
     *          applies differential drive kinematics, and updates the global position estimate.
     *
     *          **Algorithm:**
     *          1. Read current distances from both wheels
     *          2. Compute deltas (distance traveled since last update)
     *          3. Compute average forward distance and heading change
     *          4. Apply rotation matrix to update x, y, theta
     *          5. Update velocity (decimated to reduce noise)
     *          6. Store updated state thread-safely
     *
     *          **Kinematic Equations:**
     *          ```
     *          delta_left = currentLeft - lastLeft
     *          delta_right = currentRight - lastRight
     *          
     *          delta_distance = (delta_right + delta_left) / 2
     *          delta_theta = (delta_right - delta_left) / wheelSpacing
     *          
     *          x += delta_distance * cos(theta)
     *          y += delta_distance * sin(theta)
     *          theta += delta_theta
     *          
     *          rightSpeed = delta_right * frequency
     *          leftSpeed = delta_left * frequency
     *          ```
     *
     * @note **Called Periodically:** Invoked by the background task at the configured frequency
     *       (e.g., 100-1000 Hz). Do not call directly; let the task handle it.
     *
     * @note **Velocity Decimation:** Velocity updates (rightEncoderSpeed, leftEncoderSpeed)
     *       are computed at a lower frequency (max 100 Hz) to reduce quantization noise
     *       from discrete encoder counts. Position integration happens every call (full frequency).
     *
     * @note **No Blocking:** This function should complete quickly (< 1 ms) to avoid
     *       jittering the periodic task. It does not perform I/O or acquire locks (except
     *       internally through SetState).
     *
     * @note **Sensor Error Tolerance:** If encoder reads fail, the function uses the last
     *       valid distance and continues (graceful degradation rather than crash).
     *
     * @warning **Precision Loss:** For very small displacements (< 1 cm per call), quantization
     *          errors from discrete encoder counts dominate. At 1000 Hz, this is expected behavior.
     *          If precision is critical, reduce update frequency or use higher-resolution encoders.
     *
     * @warning **Slip Assumption:** Assumes perfect rolling (no slipping). Wheel slip causes
     *          systematic error. On slippery surfaces, odometry accuracy degrades.
     *
     * @see BaseOdometry::OdometryUpdate()
     * @see GenericDifferentialDriveOdometry for state management
     */
    void OdometryUpdate() override;

    /**
     * @brief The last recorded right wheel distance, used to compute incremental distance.
     * @details Stores the right wheel's distance at the previous OdometryUpdate call.
     *          Compared to the current distance in the next call to compute delta_right.
     * @note Updated each OdometryUpdate() call.
     */
    float _lastRightDistanceRef;

    /**
     * @brief The last recorded left wheel distance, used to compute incremental distance.
     * @details Stores the left wheel's distance at the previous OdometryUpdate call.
     *          Compared to the current distance in the next call to compute delta_left.
     * @note Updated each OdometryUpdate() call.
     */
    float _lastLeftDistanceRef;

    /**
     * @brief Counter used to decimate velocity updates to reduce quantization noise.
     * @details Velocity calculation (encoder speed) is expensive and noisy at high frequencies.
     *          This counter skips velocity calculation on many calls, computing it only every Nth call.
     *          Typical decimation: 10x (compute velocity at 100 Hz if position updates at 1000 Hz).
     * @note Incremented and checked each OdometryUpdate() call.
     */
    uint8_t _velCounter;
};

} // namespace Motion::Core::Robot