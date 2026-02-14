/**
 * @file BaseOdometry.cpp
 * @brief A speed profile generator that returns constant speed.
 */

#include "Core/Robot/Odom/DifferentialDriveOdometry.h"

namespace Motion::Core::Robot {

DifferentialDriveOdometry::DifferentialDriveOdometry(float wheelSpacing, Wheel* rightWheel, Wheel* leftWheel) :
    GenericDifferentialDriveOdometry(wheelSpacing, rightWheel, leftWheel), _lastRightDistanceRef(0.0f), _lastLeftDistanceRef(0.0f), _velCounter(0) {}

DifferentialDriveOdometry::~DifferentialDriveOdometry() {}

void DifferentialDriveOdometry::OdometryUpdate() {
    float currentDistRight = _rightWheel->getDistance();
    float currentDistLeft = _leftWheel->getDistance();

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