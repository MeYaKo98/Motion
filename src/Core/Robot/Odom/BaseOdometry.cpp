/**
 * @file BaseOdometry.cpp
 * @brief A speed profile generator that returns constant speed.
 */

#include "Core/Robot/Odom/BaseOdometry.h"

namespace Motion::Core::Robot {

BaseOdometry::BaseOdometry() 
    :_started(false), _odometryFrequency(1000), _odomTaskHandler(nullptr),
    _position({0.0f, 0.0f, 0.0f}), _positionMutex(nullptr)
{
    // Create binary semaphore (mutex) for thread-safe position access
    _positionMutex = xSemaphoreCreateMutex();
    if (_positionMutex == nullptr)
    {
        throw std::runtime_error("Failed to create position mutex");
    }
}

BaseOdometry::~BaseOdometry() {
    Stop();
    if (_positionMutex != nullptr) {
        vSemaphoreDelete(_positionMutex);
    }
}

void BaseOdometry::Start(uint16_t odometryFrequency) {
    if (_odomTaskHandler) {
        LOG_WARN("Odometry already started!");
        return;
    }
    if (odometryFrequency > 1000)
    {
        LOG_WARN("Odometry frequency set to 1000 Hz!");
        odometryFrequency = 1000;
    }
    if (1000 % odometryFrequency != 0)
    {
        odometryFrequency = 1000 / (1000 / odometryFrequency);
    }
    _odometryFrequency = odometryFrequency;
    xTaskCreate(OdometryTask, "OdometryTask", 2048, this, 10, &_odomTaskHandler);
}

void BaseOdometry::OdometryTask(void* pvParameters) {
    BaseOdometry* odom = static_cast<BaseOdometry*>(pvParameters);
    const TickType_t period = pdMS_TO_TICKS(1000/odom->_odometryFrequency);
    TickType_t lastWakeTime = xTaskGetTickCount();
    odom->_started = true;
    LOG_INFO("Odometry started at a frequency of %d Hz", odom->_odometryFrequency);
    while (true)
    {
        odom->OdometryUpdate();
        vTaskDelayUntil(&lastWakeTime, period);
    }
}

Position BaseOdometry::GetPosition() {
    // Combine null check and semaphore take for cleaner flow
    if (_positionMutex != nullptr && xSemaphoreTake(_positionMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        Position result = _position;
        xSemaphoreGive(_positionMutex);
        return result;
    }

    LOG_WARN("BaseOdometry: Failed to lock position for reading");
    return _position;
}

bool BaseOdometry::SetPosition(const Position& newPosition) {
    if (_positionMutex == nullptr) return false;

    if (xSemaphoreTake(_positionMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        _position = newPosition;
        xSemaphoreGive(_positionMutex);
        return true;
    }

    LOG_ERROR("BaseOdometry: Failed to lock position for writing");
    return false;
}

void BaseOdometry::Stop() {
    if (_odomTaskHandler != nullptr) {
        vTaskDelete(_odomTaskHandler);
        _odomTaskHandler = nullptr;
        _started = false;
        LOG_INFO("Odometry stopped");
    }
}

}