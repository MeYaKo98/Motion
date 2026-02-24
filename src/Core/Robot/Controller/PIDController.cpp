/**
 * @file PIDController.cpp
 * @brief Implementation of the PID Controller for closed-loop actuator control.
 */

#include "Core/Robot/Controller/PIDController.h"

namespace Motion::Core::Robot {

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

PIDController::~PIDController() {}

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

float PIDController::GenerateCommand(float error) {
    float command = 0;
    
    // Accumulate error for integral term
    _integral += error;
    
    // Calculate PID output
    command = _coefficient.Kp * error + _coefficient.Ki * _integral + _coefficient.Kd * (error - _lastError);
    
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