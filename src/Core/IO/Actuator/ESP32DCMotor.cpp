/**
 * @file ESP32DCMotor.cpp
 * @brief Motor hardware interface for ESP32.
 */

#include "Core/IO/Actuator/ESP32DCMotor.h"

namespace Motion::Core::IO {

ESP32DCMotor::ESP32DCMotor(const char* name, DCMotorConfig config) : GenericDCMotor(name, config) {}

bool ESP32DCMotor::Start()
{
    pinMode(this->_config.pinA, OUTPUT);
    pinMode(this->_config.pinB, OUTPUT);
    return true;
}

ESP32DCMotor::~ESP32DCMotor() {
    Stop();
}

void ESP32DCMotor::Stop() {
    SetCommand(0.0f);
}

void ESP32DCMotor::SendCommand(float command) {
    if (command > 0)
    {
        if (command>1.0f) command = 1.0f;
        analogWrite(this->_config.pinA, command * 255);
        analogWrite(this->_config.pinB, 0);
    }
    else
    {
        if (command<-1.0f) command = -1.0f;
        analogWrite(this->_config.pinB, -command * 255);
        analogWrite(this->_config.pinA, 0);
    }
}

}