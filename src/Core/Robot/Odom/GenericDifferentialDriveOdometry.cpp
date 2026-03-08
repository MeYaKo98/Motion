/**
 * @file BaseOdometry.cpp
 * @brief A speed profile generator that returns constant speed.
 */

#include "Motion/Core/Robot/Odom/GenericDifferentialDriveOdometry.h"

namespace Motion::Core::Robot {

GenericDifferentialDriveOdometry::GenericDifferentialDriveOdometry(float wheelSpacing, WheelHandle& rightWheelHandle, WheelHandle& leftWheelHandle)
    : _wheelSpacing(abs(wheelSpacing)), _rightWheelHandle(rightWheelHandle), _leftWheelHandle(leftWheelHandle), _state({0.0f, 0.0f, 0.0f, 0.0f})
{
    _stateMutex = xSemaphoreCreateMutex();
    if (_stateMutex == nullptr) throw std::runtime_error("Failed to create state mutex");
}

GenericDifferentialDriveOdometry::~GenericDifferentialDriveOdometry()
{
    if (_stateMutex != nullptr)
        vSemaphoreDelete(_stateMutex);
}

GenericDifferentialDriveOdometry::DifferentialDriveState GenericDifferentialDriveOdometry::GetState()
{
    if (_stateMutex != nullptr && xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(5)) == pdTRUE)
    {
        DifferentialDriveState result = _state;
        xSemaphoreGive(_stateMutex);
        return result;
    } 
    
    LOG_WARN("Odometry: Failed to lock state for reading");
    return _state;
}

bool GenericDifferentialDriveOdometry::SetState(const GenericDifferentialDriveOdometry::DifferentialDriveState& newState)
{
    if (_stateMutex == nullptr) return false;

    if (xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(5)) == pdTRUE)
    {
        _state = newState;
        xSemaphoreGive(_stateMutex);
        return true;
    }
    
    LOG_ERROR("Odometry: Failed to lock state for writing");
    return false;
}

}