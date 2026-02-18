/**
 * @file PIDController.cpp
 * @brief Implementation of the PID Controller for closed-loop actuator control.
 */

#include "Core/Robot/Controller/PIDController.h"

namespace Motion::Core::Robot {

/**
 * @brief Factory constructor for PIDController.
 * @details Creates a new PIDController with validation of all parameters.
 *          Ensures the controller is properly configured before instantiation.
 *
 * @param coefficient PID coefficients (Kp, Ki, Kd). All must be non-negative.
 * @param max Maximum output command. Must be >= min.
 * @param min Minimum output command. Must be <= max.
 * @return PIDControllerHandle A shared pointer to the new controller instance.
 * @throws std::invalid_argument if Kp, Ki, or Kd is negative.
 * @throws std::invalid_argument if Kp == 0 AND Ki == 0 AND Kd == 0 (no active control).
 * @throws std::invalid_argument if max < min.
 */
PIDControllerHandle PIDController::Create(PIDCoefficient coefficient, float max, float min)
{
    if (coefficient.Kp < 0 || coefficient.Ki < 0 || coefficient.Kd < 0) 
        throw std::invalid_argument("Coefficients must be positive");
    if (coefficient.Kp == 0 && coefficient.Ki == 0 && coefficient.Kd == 0) 
        throw std::invalid_argument("At least one coefficient must be non-zero");
    if (max < min) 
        throw std::invalid_argument("Max must be greater than or equal to min");
    return PIDControllerHandle(new PIDController(coefficient, max, min));
}

/**
 * @brief Destructor for PIDController.
 * @details Performs cleanup. No dynamic memory is allocated by this class,
 *          so the destructor is essentially empty.
 */
PIDController::~PIDController() {}

/**
 * @brief Constructs a PIDController instance.
 * @details Initializes all members with the provided coefficients and limits.
 *
 * @param coefficient PID coefficients (already validated by Create()).
 * @param max Maximum output value.
 * @param min Minimum output value.
 */
PIDController::PIDController(PIDCoefficient coefficient, float max, float min) :
    _coefficient(coefficient), _integral(0), _lastError(0), BaseController(max, min)  {}

/**
 * @brief Resets the internal PID state.
 * @details Clears accumulated integral error and previous error for a fresh start.
 *          Called before each new motion sequence.
 */
void PIDController::Reset() {
    _lastError = 0;
    _integral = 0;
}

/**
 * @brief Generates the PID control command.
 * @details Implements the standard PID equation:
 *          command = (Kp * error) + (Ki * integral_sum) + (Kd * (last_error - error))
 *
 *          Then clamps the output to [min, max] to prevent saturation.
 *
 * @param error Current error value (setpoint - actual).
 * @return float Control command, clamped to [_minCommand, _maxCommand].
 *
 * Algorithm:
 *  1. Accumulate integral: _integral += error
 *  2. Compute P, I, D terms
 *  3. Sum the terms: command = (Kp * error) + (Ki * _integral) + (Kd * (_lastError - error))
 *  4. Clamp output to valid range
 *  5. Save current error as last error for next iteration
 *  6. Return clamped command
 */
float PIDController::GenerateCommand(float error) {
    float command = 0;
    
    // Accumulate error for integral term
    _integral += error;
    
    // Calculate PID output
    command = _coefficient.Kp * error + _coefficient.Ki * _integral + _coefficient.Kd * (_lastError - error);
    
    // Update last error for derivative calculation
    _lastError = error;
    
    // Clamp output to valid range
    if (command > _maxCommand)
        command = _maxCommand;
    if (command < _minCommand)
        command = _minCommand;
    
    return command;
}

} // namespace Motion::Core::Robot