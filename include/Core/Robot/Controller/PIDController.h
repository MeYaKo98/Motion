/**
 * @file PIDController.h
 * @brief A PID controller for closed loop actuator control.
 */

#pragma once

#include "Core/Robot/Controller/BaseController.h"

namespace Motion::Core::Robot {

/**
 * @brief Configuration coefficients for the PID controller.
 */
struct PIDCoefficient {
    /** @brief Proportional gain constant. */
    float Kp;
    /** @brief Derivative gain constant. */
    float Kd;
    /** @brief Integral gain constant. */
    float Ki;
};

/**
 * @brief A Proportional-Integral-Derivative (PID) controller for closed-loop actuator control.
 * @details This class implements a standard PID control loop. It calculates an error value as the difference
 *          between a desired setpoint and a measured process variable, then applies a correction based on
 *          proportional, integral, and derivative terms.
 */
class PIDController : public BaseController {
public:
    /**
     * @brief Constructs a new PID Controller object.
     * @param coefficient The PID coefficients (Kp, Ki, Kd) to be used for the control loop.
     * @param max The maximum allowable output value (saturation limit).
     * @param min The minimum allowable output value (saturation limit).
     */
    explicit PIDController(PIDCoefficient coefficient, float max, float min);

    /**
     * @brief Destroys the PID Controller object.
     */
    ~PIDController() override;

    /**
     * @brief Resets the controller's internal state.
     * @details Resets the accumulated integral error and the last recorded error to zero.
     *          This should be called before starting a new control sequence to prevent
     *          unexpected behavior or "jumps" caused by stale state data.
     */
    void Reset() override;

    /**
     * @brief Generates the command value based on the input error.
     * @param error The difference between the target setpoint and the actual measured value.
     * @return float The computed control output, clamped within the [min, max] range.
     * @note The output calculation typically follows: Output = (Kp * error) + (Ki * integral) + (Kd * derivative).
     */
    float GenerateCommand(float error) override;

protected:
    /** @brief The PID coefficients used for control calculations. */
    PIDCoefficient _coefficient;
    /** @brief The error value from the previous cycle, used to calculate the derivative term. */
    float _lastError;
    /** @brief The accumulated error over time, used for the integral term. */
    float _integral;
};

} // namespace Motion::Core::Robot