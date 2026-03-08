#pragma once

/*--------------Communication--------------*/
#include "Motion/Core/IO/Communication/BaseChannel.h"
#include "Motion/Core/IO/Communication/GenericTCP.h"
#include "Motion/Core/IO/Communication/GenericSerial.h"

#ifdef ESP32
#include "Motion/Core/IO/Communication/ESP32TCP.h"
#include "Motion/Core/IO/Communication/ESP32Serial.h"
#endif


/*--------------Sensor--------------*/
#include "Motion/Core/IO/Sensor/BaseSensor.h"
#include "Motion/Core/IO/Sensor/GenericEncoder.h"

#ifdef ESP32
#include "Motion/Core/IO/Sensor/ESP32Encoder.h"
#endif

/*--------------Actuator--------------*/
#include "Motion/Core/IO/Actuator/BaseActuator.h"
#include "Motion/Core/IO/Actuator/GenericMotor.h"

#ifdef ESP32
#include "Motion/Core/IO/Actuator/ESP32DCMotor.h"
#endif
