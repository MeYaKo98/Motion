/**
 * @file BaseOdometry.cpp
 * @brief A speed profile generator that returns constant speed.
 */

#include "Core/Robot/Odom/GenericDifferentialDriveOdometry.h"

namespace Motion::Core::Robot {

GenericDifferentialDriveOdometry::GenericDifferentialDriveOdometry(float wheelSpacing, Wheel* rightWheel, Wheel* leftWheel) :
    _wheelSpacing(wheelSpacing), _rightWheel(rightWheel), _leftWheel(leftWheel), _state({0.0f, 0.0f, 0.0f, 0.0f}) {
        if (rightWheel == nullptr) LOG_ERROR("Right Wheel Pointer can not be NULL");
        if (leftWheel == nullptr) LOG_ERROR("Left Wheel Pointer can not be NULL");
        _stateMutex = xSemaphoreCreateMutex();
        if (_stateMutex == nullptr) {
            LOG_ERROR("Failed to create state mutex");
        }
}

GenericDifferentialDriveOdometry::~GenericDifferentialDriveOdometry() {
    if (_stateMutex != nullptr) {
        vSemaphoreDelete(_stateMutex);
    }
}

GenericDifferentialDriveOdometry::DifferentialDriveState GenericDifferentialDriveOdometry::GetState() {
    if (_stateMutex != nullptr && xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        DifferentialDriveState result = _state;
        xSemaphoreGive(_stateMutex);
        return result;
    } 
    
    LOG_WARN("Odometry: Failed to lock state for reading");
    return _state;
}

bool GenericDifferentialDriveOdometry::SetState(const GenericDifferentialDriveOdometry::DifferentialDriveState& newState){
if (_stateMutex == nullptr) return false;

    if (xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        _state = newState;
        xSemaphoreGive(_stateMutex);
        return true;
    }
    
    LOG_ERROR("Odometry: Failed to lock state for writing");
    return false;
}

}