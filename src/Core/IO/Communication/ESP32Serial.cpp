#include "Core/IO/Communication/ESP32Serial.h"

namespace Motion::Core::IO {

ESP32Serial::ESP32Serial(HardwareSerial& serial, uint32_t baudRate) 
    : GenericSerial(baudRate), _serial(serial) {}

ESP32Serial::~ESP32Serial()
{
    Stop();
}

ESP32SerialHandle ESP32Serial::Create(HardwareSerial& serial, uint32_t baudRate)
{
    if (baudRate == 0)
    {
        Serial.begin(912600);
        Serial.println("Baud rate cannot be zero");
        throw std::invalid_argument("Baud rate cannot be zero");
    }
    return ESP32SerialHandle(new ESP32Serial(serial, baudRate));
}

bool ESP32Serial::Start()
{
    if (!_started)
    {
        _serial.begin(_baudRate);
        _started = true;
    }
    return _started;
}

size_t ESP32Serial::Send(const uint8_t* data, size_t length)
{
    if (!_started || data == nullptr || length == 0) {
        return 0;
    }
    return _serial.write(data, length);
}

size_t ESP32Serial::Read(uint8_t* buffer, size_t bufferSize)
{
    if (!_started || buffer == nullptr || bufferSize == 0)
    {
        return 0;
    }
    size_t available = _serial.available();
    if (available == 0)
        return 0;
    size_t toRead = (available > bufferSize) ? bufferSize : available;
    return _serial.readBytes(buffer, toRead);
}

bool ESP32Serial::IsConnected()
{
    return _started;
}

void ESP32Serial::Stop()
{
    if (_started)
    {
        _serial.end();
        _started = false;
    }
}

} // namespace Motion::Core::IO