/**
 * @file DifferentialDriveNavigation.cpp
 * @brief A Navigation Class for differential drive robot.
 */

#include "Motion/Core/Robot/Navigation/DifferentialDriveNavigation.h"

namespace Motion::Core::Robot {

DifferentialDriveNavigation::DifferentialDriveNavigation (GenericDifferentialDriveOdometryHandle odometryHandle, BaseProfileGeneratorHandle profileGeneratorHandle, BaseStopConditionHandle stopConditionHandle, DifferentialDriveMotorConfig motorConfig)
    : GenericDifferentialDriveNavigation(odometryHandle, profileGeneratorHandle, stopConditionHandle, motorConfig) {}

DifferentialDriveNavigation::~DifferentialDriveNavigation() {}

DifferentialDriveNavigationHandle DifferentialDriveNavigation::Create(GenericDifferentialDriveOdometryHandle odometryHandle, BaseProfileGeneratorHandle profileGeneratorHandle, BaseStopConditionHandle stopConditionHandle, DifferentialDriveMotorConfig motorConfig)
{
    if (odometryHandle == nullptr) throw std::invalid_argument("odometryHandle cannot be nullptr");
    if (profileGeneratorHandle == nullptr) throw std::invalid_argument("profileGeneratorHandle cannot be nullptr");
    if (stopConditionHandle == nullptr) throw std::invalid_argument("stopConditionHandle cannot be nullptr");
    if (motorConfig.rightMotorHandle == nullptr) throw std::invalid_argument("rightMotorHandle cannot be nullptr");
    if (motorConfig.leftMotorHandle == nullptr) throw std::invalid_argument("leftMotorHandle cannot be nullptr");
    if (motorConfig.rightControllerHandle == nullptr) throw std::invalid_argument("rightController cannot be nullptr");
    if (motorConfig.leftControllerHandle == nullptr) throw std::invalid_argument("leftController cannot be nullptr");
    return DifferentialDriveNavigationHandle(new DifferentialDriveNavigation(odometryHandle, profileGeneratorHandle, stopConditionHandle, motorConfig));
}

void DifferentialDriveNavigation::Move(float distance)
{
    LOG_TRACE("Move called with distance: %f", distance);
    
    // Reset all controllers and stop condition for a fresh motion sequence
    _stopConditionHandle->Reset();
    _motorConfig.rightControllerHandle->Reset();
    _motorConfig.leftControllerHandle->Reset();
    
    // Initialize odometry baseline for relative distance tracking
    auto startState = _odometryHandle->GetState();
    float startDistance = (startState.rightDistance + startState.leftDistance) * 0.5f;
    float startOrientation = startState.rightDistance - startState.leftDistance;
    
    // Generate the velocity profile for this distance
    _profileGeneratorHandle->GenerateProfile(distance);
    
    float progressDistance = 0.0f;
    float orientationError = 0.0f;
    float goalV = 0.0f;
    float goalVr = 0.0f, goalVl = 0.0f;
    float commandR = 0.0f, commandL = 0.0f;
    
    TickType_t lastWakeTime = xTaskGetTickCount();
    
    // Main control loop: continue until stop condition is met
    while (!_stopConditionHandle->ShouldExit(distance - progressDistance))
    {
        // Read current encoder state
        auto currentState = _odometryHandle->GetState();
        
        // Calculate progress distance traveled (average of both wheels)
        progressDistance = (currentState.rightDistance + currentState.leftDistance) * 0.5f - startDistance;
        
        // Generate velocity setpoint from the motion profile based on progress
        goalV = _profileGeneratorHandle->CalculateValue(progressDistance);
        
        // Initialize goal velocities (both wheels same for straight motion)
        goalVr = goalV;
        goalVl = goalV;
        
        // Calculate orientation error (difference between wheel distances)
        orientationError = startOrientation - (currentState.rightDistance - currentState.leftDistance);
        
        // Apply orientation correction
        goalVr += orientationError;
        goalVl -= orientationError;
        
        // Generate motor commands using controllers with feedback (encoder speeds)
        commandR = _motorConfig.rightControllerHandle->GenerateCommand(goalVr, currentState.rightEncoderSpeed);
        commandL = _motorConfig.leftControllerHandle->GenerateCommand(goalVl, currentState.leftEncoderSpeed);
        
        //LOG_TRACE("d_r:%f, d_l:%f, v_r:%f, v_l:%f, c_r:%f, c_l:%f", currentState.rightDistance, currentState.leftDistance, currentState.rightEncoderSpeed, currentState.leftEncoderSpeed, commandR, commandL);
        //LOG_TRACE("c:%f, v_r:%f, v_l:%f, cr:%f, cl:%f", goalV, currentState.rightEncoderSpeed, currentState.leftEncoderSpeed, commandR, commandL);

        // Send commands to motors
        _motorConfig.rightMotorHandle->SetCommand(commandR);
        _motorConfig.leftMotorHandle->SetCommand(commandL);
        
        // Wait until next control iteration (10 ms period = 100 Hz control rate)
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10));
    }
    
    // Stop motors once the goal is reached
    _motorConfig.rightMotorHandle->SetCommand(0.0f);
    _motorConfig.leftMotorHandle->SetCommand(0.0f);
    
    LOG_TRACE("Move completed for distance: %f", distance);
}

void DifferentialDriveNavigation::Turn(float angle)
{
    LOG_TRACE("Turn called with angle: %f", angle);
    
    // Reset all controllers and stop condition for a fresh motion sequence
    _stopConditionHandle->Reset();
    _motorConfig.rightControllerHandle->Reset();
    _motorConfig.leftControllerHandle->Reset();
    
    // Initialize odometry baseline for relative rotation tracking
    auto startState = _odometryHandle->GetState();
    float startRightDistance = startState.rightDistance;
    float startLeftDistance = startState.leftDistance;
    float startDistance = startRightDistance + startLeftDistance;
    float goalOrientation = angle*_odometryHandle->_wheelSpacing*0.5f;
    
    // Generate the velocity profile for this turning distance
    // Use the angle directly (with sign) as the profile distance
    _profileGeneratorHandle->GenerateProfile(goalOrientation);
    
    float progressDistance = 0.0f;
    float distanceError = 0.0f;
    float goalV = 0.0f;
    float goalVr = 0.0f, goalVl = 0.0f;
    float commandR = 0.0f, commandL = 0.0f;
    
    TickType_t lastWakeTime = xTaskGetTickCount();
    
    // Main control loop: continue until stop condition is met
    while (!_stopConditionHandle->ShouldExit(goalOrientation - progressDistance))
    {
        // Read current encoder state
        auto currentState = _odometryHandle->GetState();
        
        // Calculate progress as the difference between wheel deltas minus start orientation, multiplied by 0.5
        float rightDelta = currentState.rightDistance - startRightDistance;
        float leftDelta = currentState.leftDistance - startLeftDistance;
        progressDistance = (rightDelta - leftDelta) * 0.5f;
        
        // Generate velocity setpoint from the motion profile based on progress
        goalV = _profileGeneratorHandle->CalculateValue(progressDistance);
        
        // Initialize goal velocities (both wheels same for straight motion)
        goalVr = goalV;
        goalVl = -goalV;

        // Calculate orientation error (difference between wheel distances)
        distanceError = startDistance - (currentState.rightDistance + currentState.leftDistance);
        
        // Apply orientation correction
        goalVr += distanceError;
        goalVl += distanceError;

        // For turning: right wheel forward, left wheel backward (opposite directions)
        commandR = _motorConfig.rightControllerHandle->GenerateCommand(goalVr, currentState.rightEncoderSpeed);
        commandL = _motorConfig.leftControllerHandle->GenerateCommand(goalVl, currentState.leftEncoderSpeed);

        //LOG_TRACE("d_r:%f, d_l:%f, v_r:%f, v_l:%f, c_r:%f, c_l:%f", currentState.rightDistance, currentState.leftDistance, currentState.rightEncoderSpeed, currentState.leftEncoderSpeed, commandR, commandL);
        //LOG_TRACE("c:%f, v_r:%f, v_l:%f, cr:%f, cl:%f", goalV, currentState.rightEncoderSpeed, currentState.leftEncoderSpeed, commandR, commandL);

        // Send commands to motors
        _motorConfig.rightMotorHandle->SetCommand(commandR);
        _motorConfig.leftMotorHandle->SetCommand(commandL);
        
        // Wait until next control iteration (10 ms period = 100 Hz control rate)
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10));
    }
    
    // Stop motors once the goal is reached
    _motorConfig.rightMotorHandle->SetCommand(0.0f);
    _motorConfig.leftMotorHandle->SetCommand(0.0f);
    
    LOG_TRACE("Turn completed for angle: %f", angle);
}

void DifferentialDriveNavigation::MoveTo(float x, float y)
{
    LOG_TRACE("MoveTo called with x: %f, y: %f", x, y);
    Position position = _odometryHandle->GetPosition();
    float dx = x - position.x;
    float dy = y - position.y;
    float orientation = atan2(dy, dx);
    float distance = sqrt(dx*dx + dy*dy);
    Orient(orientation);
    vTaskDelay(pdMS_TO_TICKS(100));
    Move(distance);
}

void DifferentialDriveNavigation::Orient(float angle)
{
    LOG_TRACE("Orient called with angle: %f", angle);
    Position position = _odometryHandle->GetPosition();
    float goal = angle - position.theta;
    if (goal>M_PI) goal -= 2*M_PI;
    if (goal<-M_PI) goal += 2*M_PI;
    return Turn(goal);
}

}