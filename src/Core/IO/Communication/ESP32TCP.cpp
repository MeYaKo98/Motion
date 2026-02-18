#include "Core/IO/Communication/ESP32TCP.h"

namespace Motion::Core::IO {

ESP32TCP::ESP32TCP(uint16_t port) 
    : GenericTCP(port), _server(port) {}

ESP32TCP::~ESP32TCP()
{
    Stop();
}

ESP32TCPHandle ESP32TCP::Create(uint16_t port)
{
    return ESP32TCPHandle(new ESP32TCP(port));
}

bool ESP32TCP::Start()
{
    if (WiFi.getMode() == WIFI_OFF)
    {
        return false; 
    }

    if (!_started)
    {
        _server.begin();
        _started = true;
    }
    return _started;
}

void ESP32TCP::Stop()
{
    if (_client.connected())
    {
        _client.stop();
    }
    _server.end();
    _started = false;
}

bool ESP32TCP::IsConnected()
{
    //check if started
    if (!_started) return false;
    //checkwifi
    bool wifiReady = (WiFi.getMode() == WIFI_AP) || (WiFi.getMode() == WIFI_AP_STA) || //case of access point
                     (WiFi.getMode() == WIFI_STA && WiFi.status() == WL_CONNECTED); //case of wifi
    if (!wifiReady) return false;
    //check client(s)
    if (_server.hasClient()) {
        if (_client && _client.connected())
        {
            //drop new conneciton requests if one client already connected
            _server.available().stop(); 
        }
        else
        {
            _client = _server.available();
        }
    }

    return _client && _client.connected();
}

size_t ESP32TCP::Send(const uint8_t* data, size_t length)
{
    if (!IsConnected() || data == nullptr) return 0;
    return _client.write(data, length);
}

size_t ESP32TCP::Read(uint8_t* buffer, size_t bufferSize)
{
    if (!IsConnected() || buffer == nullptr) return 0;
    
    size_t available = _client.available();
    if (available == 0) return 0;

    size_t toRead = (available > bufferSize) ? bufferSize : available;
    return _client.read(buffer, toRead);
}

} // namespace Motion::Core::IO