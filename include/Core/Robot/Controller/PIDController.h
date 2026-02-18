/**
 * @file PIDController.h
 * @brief A PID (Proportional-Integral-Derivative) controller implementation.
 * @details Implements the industry-standard PID control algorithm, widely used in robotics for
 *          speed regulation, position tracking, and general closed-loop control.
 */

#pragma once

#include "Core/Robot/Controller/BaseController.h"

namespace Motion::Core::Robot {

/**
 * @brief Configuration coefficients for the PID controller.
 * @details Holds the three gain constants that tune the PID controller's behavior.
 *          Proper tuning of these values is critical for system stability and performance.
 */
struct PIDCoefficient {
    /** @brief Proportional gain constant (Kp).
     *  Higher Kp increases responsiveness but may cause oscillation.
     *  Typical range: 0.0 to 100.0 (application-dependent).
     *  Must be >= 0.
     */
    float Kp;
    
    /** @brief Derivative gain constant (Kd).
     *  Increases damping and reduces overshoot.
     *  Typical range: 0.0 to 10.0 (application-dependent).
     *  Must be >= 0.
     */
    float Kd;
    
    /** @brief Integral gain constant (Ki).
     *  Eliminates steady-state error but may cause slow response.
     *  Can lead to integral windup if not properly saturated.
     *  Typical range: 0.0 to 10.0 (application-dependent).
     *  Must be >= 0.
     */
    float Ki;
};

/**
 * @brief A Proportional-Integral-Derivative (PID) controller for closed-loop actuator control.
 * @details This class implements a standard PID control loop. The algorithm computes:
 *          Output = (Kp * error) + (Ki * integral) + (Kd * (error_last - error))
 *          
 *          The three terms work together:
 *          - **Proportional (P):** responds to current error
 *          - **Integral (I):** responds to accumulated error over time
 *          - **Derivative (D):** responds to rate of change of error (damping)
 *          
 *          The output is clamped to [min, max] to prevent saturation and integral windup.
 * @note **Tuning:** PID parameters should be tuned using methods like Ziegler-Nichols
 *       or empirical trial-and-error to achieve desired response (fast, stable, minimal overshoot).
 * @see BaseController
 */
class PIDController;

using PIDControllerHandle = ControllerPointer(PIDController);

class PIDController : public BaseController {
public:

    /**
     * @brief Factory method to create a PIDController instance with validation.
     * @details Creates a new PIDController with the specified coefficients and limits.
     *          Validates all parameters before creation to prevent invalid configurations.
     * @param coefficient The PID gain coefficients (Kp, Ki, Kd). All must be non-negative.
     * @param max The maximum command output value.
     * @param min The minimum command output value. Must be <= max.
     * @return PIDControllerHandle A shared pointer to the newly created controller.
     * @throws std::invalid_argument if any coefficient is negative.
     * @throws std::invalid_argument if all coefficients are zero (no active control).
     * @throws std::invalid_argument if max < min.
     * @note **Usage Pattern:**
     *       ```cpp
     *       auto controller = PIDController::Create({1.0f, 0.1f, 0.5f}, 255.0f, -255.0f);
     *       ```
     * @warning Passing invalid parameters will throw an exception. Always wrap in try-catch if
     *          parameters come from external sources.
     */
    static PIDControllerHandle Create(PIDCoefficient coefficient, float max, float min);

    /**
     * @brief Destroys the PID Controller object.
     * @details Cleans up any dynamically allocated resources.
     *          Calls the destructor of the base controller.
     */
    ~PIDController() override;

    /**
     * @brief Resets the controller's internal state.
     * @details Resets the accumulated integral error and the last recorded error to zero.
     *          This should be called before starting a new control sequence to prevent
     *          unexpected behavior or "jumps" caused by stale state data.
     *          
     *          After reset:
     *          - `_lastError` = 0.0f
     *          - `_integral` = 0.0f
     *          
     *          The first call to `GenerateCommand()` after reset will produce:
     *          Output = Kp * error (I and D terms are zero)
     *
     * @return void
     * @note **When to Call:** Call this before each new motion sequence or after stopping.
     *       For example, call before moving to a new waypoint or changing the setpoint drastically.
     * @note **Side Effect:** The first output after reset may differ significantly from steady-state
     *       due to zero I and D terms. Allow a brief settling period.
     * @warning **Timing:** Do not call Reset() during active control; it will cause a discontinuity.
     */
    void Reset() override;

    /**
     * @brief Generates the command value based on the input error.
     * @details Calculates and returns the PID control output based on the current error.
     *          The algorithm performs the following steps:
     *          1. Accumulate error: `_integral += error`
     *          2. Calculate output: `Output = (Kp * error) + (Ki * _integral) + (Kd * (_lastError - error))`
     *          3. Saturation clamping: clamp output to [_minCommand, _maxCommand]
     *          4. Update state: `_lastError = error`
     *          5. Return clamped output
     *
     * @param error The difference between the target setpoint and the actual measured value.
     *              Positive error typically indicates the actuator should increase output.
     *              Negative error indicates the actuator should decrease output.
     *              Error = setpoint - actual.
     * @return float The computed control output, clamped within the [min, max] range.
     *               This value is ready to send directly to the actuator.
     *               
     *               Example outputs:
     *               - Motor speed: [-255, 255] for PWM
     *               - Servo angle: [0, 180] for degrees
     *               - Normalized: [-1.0, 1.0] for normalized output
     *
     * @note **Output Calculation:** 
     *       ```
     *       output = (Kp * error) + (Ki * integral) + (Kd * (error_last - error))
     *       output = clamp(output, min, max)
     *       ```
     *
     * @note **Derivative Term Sign:** The derivative term uses `(_lastError - error)` instead of
     *       `(error - _lastError)` to compute the negative rate of change. This is equivalent to
     *       `Kd * (-d(error)/dt)`, providing damping to the response.
     *
     * @note **Integral Windup Prevention:** The output is saturated before returning. This prevents
     *       unlimited accumulation of the integral term. For enhanced anti-windup, derived classes
     *       could implement conditional integral accumulation (accumulated only when output is not saturated).
     *
     * @note **Frequency Assumption:** This algorithm assumes constant sampling time (call frequency).
     *       If the call frequency varies, consider using time-normalized integral and derivative terms.
     *
     * @note **Thread-safety:** This method is **not** thread-safe. Concurrent calls will cause race
     *       conditions on `_integral` and `_lastError`. Use external synchronization if needed.
     *
     * @warning **Large Errors:** Very large error values can cause overflow in fixed-point arithmetic
     *          or loss of precision in floating-point. Consider clamping error input in calling code.
     *
     * @warning **Dead Band:** At very small error values, quantization noise from the sensor may
     *          cause the controller to oscillate around the setpoint. A dead band filter can help.
     *
     * @see Reset()
     * @see BaseController::GenerateCommand()
     */
    float GenerateCommand(float error) override;

protected:
    /**
     * @brief Constructs a new PID Controller object.
     * @details Initializes the PID controller with the specified coefficients and output limits.
     *          Note: This is a protected constructor. Use the `Create()` factory method instead.
     *
     * @param coefficient The PID coefficients (Kp, Ki, Kd) to be used for the control loop.
     *                    These should be pre-validated by the `Create()` factory.
     * @param max The maximum allowable output value (saturation limit).
     * @param min The minimum allowable output value (saturation limit).
     *
     * @post `_lastError` is initialized to 0.0f.
     *       `_integral` is initialized to 0.0f.
     *       Internal state is clean for the first control iteration.
     */
    explicit PIDController(PIDCoefficient coefficient, float max, float min);

    /** @brief The PID coefficients (Kp, Ki, Kd) used for control calculations.
     *  These values are constant after initialization.
     *  Changing them requires creating a new controller instance.
     */
    PIDCoefficient _coefficient;

    /** @brief The error value from the previous cycle, used to calculate the derivative term.
     *  Initialized to 0.0f in the constructor.
     *  Updated at the end of each `GenerateCommand()` call.
     *  Used to compute: derivative_term = Kd * (_lastError - error)
     */
    float _lastError;

    /** @brief The accumulated error over time, used for the integral term.
     *  Initialized to 0.0f in the constructor.
     *  Incremented at the beginning of each `GenerateCommand()` call by the current error.
     *  Can grow unbounded without saturation (handled by output clamping).
     *  Consider conditional accumulation (only when output is not saturated) for better anti-windup.
     *  Reset to 0.0f by calling `Reset()`.
     */
    float _integral;
};

} // namespace Motion::Core::Robot