/**
 * @file BaseController.h
 * @brief Defines the abstract base interface for closed-loop actuator controllers.
 * @details This interface standardizes the interaction with various control algorithms (PID, Fuzzy, MPC, etc.)
 *          used to generate actuator commands based on error feedback. Controllers are the core of the
 *          Motion Framework's control loop.
 */

#pragma once

#include "Core/Diagnostics/Logger.h"

#define ControllerPointer(T) std::shared_ptr<T>

namespace Motion::Core::Robot {

/**
 * @brief Abstract base class for closed-loop actuator controllers.
 * @details This class provides a common interface for generating control commands based on error input.
 *          Implementations include PID, proportional, integral, derivative, and other control algorithms.
 *          Concrete implementations must override the `GenerateCommand()` and `Reset()` methods.
 */
class BaseController {
public:
    /**
     * @brief Virtual destructor for the BaseController.
     * @details Ensures proper cleanup of derived class instances when deleted through a base class pointer.
     *          Critical for polymorphic behavior and preventing resource leaks.
     * @note All virtual methods should have virtual destructors in C++.
     */
    virtual ~BaseController() = default;

    /**
     * @brief Resets the internal state of the controller.
     * @details This method should be called before starting a new control sequence or after stopping
     *          to clear any accumulated state that could affect the next motion.
     *          Examples of state to reset:
     *          - Integral windup in PID controllers
     *          - Previous error values used for derivative calculations
     *          - Low-pass filter buffers
     *          - Any internal counters or timers
     * @note **Timing:** Call this method early in the initialization sequence, before the control loop starts.
     *       Calling it during active control may introduce discontinuities.
     * @note **Side Effects:** After reset, the first call to `GenerateCommand()` may produce an atypical
     *       output because integral and derivative terms are initialized to zero.
     * @note **Implementation Requirement:** Derived classes **must** implement this method.
     */
    virtual void Reset() = 0;

    /**
     * @brief Generates a control command based on the provided error.
     * @details This is the main control algorithm. It takes the current error (difference between
     *          setpoint and actual value) and computes the command to send to the actuator.
     *          The command should be designed to minimize the error over time.
     * @param error The difference between the setpoint and the actual measured value.
     *              Mathematically: error = setpoint - actual_value.
     *              The sign convention is important: positive error typically means "increase output",
     *              negative error means "decrease output". Document this in derived classes.
     * @return float The computed control command output.
     *               The range and interpretation depend on the implementation:
     *               - For servo motors: typically [-1.0, 1.0] (normalized)
     *               - For DC motors: [-255, 255] (PWM command)
     *               - For smart actuators: actual physical units (e.g., torque in N·m)
     * @note **Output Limiting:** Implementations should ensure the return value is clamped between
     *       `_minCommand` and `_maxCommand` to avoid saturating actuators. Saturation is critical
     *       to prevent control instability.
     * @note **Calculation Time:** This method is typically called in a high-frequency loop (100+ Hz).
     *       Implementations should minimize computation time to avoid loop delays.
     *       Avoid blocking I/O, dynamic memory allocation, or complex calculations.
     * @note **Thread-safety:** This method is **not** thread-safe. If called from multiple threads,
     *       synchronize access or use separate controller instances.
     * @note **First Call:** After `Reset()`, the first call to `GenerateCommand()` may produce unexpected
     *       results due to uninitialized derivative terms. Allow a few iterations for stabilization.
     * @warning **Error Sign:** Carefully document the sign convention for error in derived classes.
     *          An inverted error sign will cause the controller to become unstable.
     * @warning **Output Limits:** Ensure `_maxCommand` >= `_minCommand`. If not, the clamping logic
     *          will fail silently or produce undefined behavior.
     */
    virtual float GenerateCommand(float error) = 0;

protected:
    /**
     * @brief Constructs a new BaseController with specified output limits.
     * @details Initializes the controller with the valid range for command outputs. This range is used
     *          to saturate the controller output and prevent windup in integral terms.
     * @param maxCommand The maximum allowable output command value. Must be >= `minCommand`.
     *                   Example: 255 for 8-bit PWM, 1.0 for normalized output.
     * @param minCommand The minimum allowable output command value. Must be <= `maxCommand`.
     *                   Example: -255 for bipolar PWM, 0.0 for unidirectional output.
     * @note **Range Validity:** The derived implementation is responsible for enforcing these limits
     *       within `GenerateCommand()`. The base class only stores these values.
     * @note **Saturation:** Output saturation is essential to prevent integral windup and control instability.
     *       Always clamp the output within [minCommand, maxCommand] before returning.
     * @warning **Invalid Ranges:** If `maxCommand < minCommand`, subsequent clamping will be incorrect.
     *          Consider adding validation in derived class constructors.
     */
    BaseController(float maxCommand, float minCommand) : _maxCommand(maxCommand), _minCommand(minCommand)  {}

    /** 
     * @brief The maximum allowable output command.
     * @details Used to saturate the controller output and prevent windup.
     *          Access this in derived classes to implement output clamping.
     */
    float _maxCommand;
    
    /** 
     * @brief The minimum allowable output command.
     * @details Used to saturate the controller output and prevent windup.
     *          Access this in derived classes to implement output clamping.
     */
    float _minCommand;
};

using BaseControllerHandle = ControllerPointer(BaseController);

} // namespace Motion::Core::Robot