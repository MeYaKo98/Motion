/**
 * @file BaseOdometry.cpp
 * @brief A speed profile generator that returns constant speed.
 */

#include "Core/Robot/Odom/DifferentialDriveOdometry.h"

namespace Motion::Core::Robot {

DifferentialDriveOdometry::DifferentialDriveOdometry(float wheelSpacing, WheelHandle& rightWheelHandle, WheelHandle& leftWheelHandle) :
    GenericDifferentialDriveOdometry(wheelSpacing, rightWheelHandle, leftWheelHandle),
    _lastRightDistanceRef(0.0f), _lastLeftDistanceRef(0.0f), _velCounter(0) {}

DifferentialDriveOdometry::~DifferentialDriveOdometry() {}

DifferentialDriveOdometryHandle DifferentialDriveOdometry::Create(float wheelSpacing, WheelHandle& rightWheelHandle, WheelHandle& leftWheelHandle)
{
    if (rightWheelHandle == nullptr) throw std::invalid_argument("RightWheelHandle can not be NULL");
    if (leftWheelHandle == nullptr) throw std::invalid_argument("LeftWheelHandle can not be NULL");
    if (wheelSpacing == 0.0f) throw std::invalid_argument("WheelSpacing must not be 0.0f");
    return DifferentialDriveOdometryHandle(new DifferentialDriveOdometry(wheelSpacing, rightWheelHandle, leftWheelHandle));
}

void DifferentialDriveOdometry::OdometryUpdate()
{
    float currentDistRight = _rightWheelHandle->getDistance();
    float currentDistLeft = _leftWheelHandle->getDistance();

    DifferentialDriveState currentState = GetState();
    Position currentPosition = GetPosition();

    // ===== Position Update / Odometry =====
    float dLeft = currentDistLeft - currentState.leftDistance;
    float dRight = currentDistRight - currentState.rightDistance;
    float dS = (dRight + dLeft) / 2.0f;
    float dTheta = (dRight - dLeft) / _wheelSpacing;

    float avgTheta = currentPosition.theta + (dTheta / 2.0f);
    
    currentPosition.x += dS * cos(avgTheta);
    currentPosition.y += dS * sin(avgTheta);
    currentPosition.theta += dTheta;

    if (currentPosition.theta > M_PI)
        currentPosition.theta -= 2.0f * M_PI;
    if (currentPosition.theta < -M_PI)
        currentPosition.theta += 2.0f * M_PI;

    //set updated position
    SetPosition(currentPosition);


    // ===== State Update / Velocity Estimation =====
    currentState.rightDistance = currentDistRight;
    currentState.leftDistance = currentDistLeft;

    _velCounter++;
    if (_velCounter >= _odometryFrequency/100)
    {
        currentState.rightEncoderSpeed = (currentDistRight - _lastRightDistanceRef)/(1.0f*_velCounter/_odometryFrequency);
        currentState.leftEncoderSpeed = (currentDistLeft - _lastLeftDistanceRef)/(1.0f*_velCounter/_odometryFrequency);
        _lastRightDistanceRef = currentDistRight;
        _lastLeftDistanceRef = currentDistLeft;
        _velCounter = 0;
    }
    SetState(currentState);
}

}