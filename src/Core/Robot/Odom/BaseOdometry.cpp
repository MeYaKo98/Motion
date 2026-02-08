/**
 * @file BaseOdometry.cpp
 * @brief A speed profile generator that returns constant speed.
 */

#include "Core\Robot\Odom\BaseOdometry.h"

namespace Motion::Core::Robot {

BaseOdometry::BaseOdometry() : _started(false), _odometryFrequency(1000), _odomTaskHandler(nullptr), _position({0.0f, 0.0f, 0.0f}) {}

BaseOdometry::~BaseOdometry() {
    Stop();
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

Position BaseOdometry::GetPosition(){
    return _position;
}

bool BaseOdometry::SetPosition(Position newPosition){
    if (_started) {
        LOG_ERROR("Odometry already started. SetPosition aborted!");
        return false;
    }
    _position = newPosition;
    return true;
}

void BaseOdometry::Stop() {
    if (_odomTaskHandler != nullptr)
    {
        vTaskDelete(_odomTaskHandler);
        _odomTaskHandler = nullptr;
        _started = false;
        LOG_ERROR("Odometry stopped");
    }
}

}