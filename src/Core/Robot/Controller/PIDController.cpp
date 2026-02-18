/**
 * @file PIDController.cpp
 * @brief  A closed loop PID controller for Actuator.
 */

#include "Core/Robot/Controller/PIDController.h"

namespace Motion::Core::Robot {

PIDController::PIDController(PIDCoefficient coefficient, float max, float min) :
    _coefficient(coefficient), _integral(0), _lastError(0), BaseController (max, min)  {}

PIDController::~PIDController() {}

PIDControllerHandle PIDController::Create(PIDCoefficient coefficient, float max, float min)
{
    if (coefficient.Kp < 0 || coefficient.Ki < 0 || coefficient.Kd < 0) throw std::invalid_argument("Coefficents must be positive");
    if (coefficient.Kp == 0 && coefficient.Ki == 0 && coefficient.Kd == 0) throw std::invalid_argument("Coefficents must not be all zeros");
    if (max < min) throw std::invalid_argument("Max must be greater than min");
    return PIDControllerHandle(new PIDController(coefficient, max, min));
}

void PIDController::Reset() {
    _lastError = 0;
    _integral = 0;
}

float PIDController::GenerateCommand(float error) {
    float command = 0;
    _integral += error;
    command = _coefficient.Kp * error + _coefficient.Ki * _integral + _coefficient.Kd * (_lastError - error);
    _lastError = error;
    if (command>_maxCommand)
        command = _maxCommand;
    if (command<_minCommand)
        command = _minCommand;
    return command;
}

} // namespace Motion::Core::Robot