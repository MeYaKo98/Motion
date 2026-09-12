/**
 * @file ESP32SerialLogging.cpp
 * @brief ESP32 logging over USB serial.
 */

#include <Motion/Core.h>

using namespace Motion::Core;
using namespace Motion::Core::IO;

constexpr uint32_t SerialBaudRate = 921600;

void setup()
{
    BaseChannelHandle loggerHandle =
        ESP32Serial::Create(Serial, SerialBaudRate);

    if (!loggerHandle)
    {
        return;
    }

    LOG_START(loggerHandle, LogLevel::TRACE);
    LOG_INFO("ESP32 serial logger started at %lu baud", SerialBaudRate);
}

void loop()
{
    LOG_TRACE("serial logging heartbeat");
    delay(1000);
}