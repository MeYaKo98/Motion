/**
 * @file BaseOdometry.cpp
 * @brief A speed profile generator that returns constant speed.
 */

#include "Core\Robot\Odom\DifferentialDriveOdometry.h"

namespace Motion::Core::Robot {

DifferentialDriveOdometry::DifferentialDriveOdometry(float wheelSpacing, Wheel* rightWheel, Wheel* leftWheel) :
    GenericDifferentialDriveOdometry(wheelSpacing, rightWheel, leftWheel), _lastRightDistanceRef(0.0f), _lastLeftDistanceRef(0.0f), _velCounter(0) {}

DifferentialDriveOdometry::~DifferentialDriveOdometry() {}

void DifferentialDriveOdometry::OdometryUpdate() {
    float currentDistRight = _rightWheel->getDistance();
    float currentDistLeft = _leftWheel->getDistance();

    float dLeft = currentDistLeft - _state.leftDistance;
    float dRight = currentDistRight - _state.rightDistance;
    float dS = (dRight + dLeft) / 2.0f;
    float dTheta = (dRight - dLeft) / _wheelSpacing;

    float avgTheta = _position.theta + (dTheta / 2.0f);
    
    _position.x += dS * cos(avgTheta);
    _position.y += dS * sin(avgTheta);
    _position.theta += dTheta;

    if (_position.theta > M_PI)
        _position.theta -= 2.0f * M_PI;
    if (_position.theta < -M_PI)
        _position.theta += 2.0f * M_PI;

    _state.rightDistance = currentDistRight;
    _state.leftDistance = currentDistLeft;

    _velCounter++;
    if (_velCounter >= _odometryFrequency/100)
    {
        _state.rightEncoderSpeed = (currentDistRight - _lastRightDistanceRef)/(1.0f*_velCounter/_odometryFrequency);
        _state.leftEncoderSpeed = (currentDistLeft - _lastLeftDistanceRef)/(1.0f*_velCounter/_odometryFrequency);
        _lastRightDistanceRef = currentDistRight;
        _lastLeftDistanceRef = currentDistLeft;
        _velCounter = 0;
    }
}

}