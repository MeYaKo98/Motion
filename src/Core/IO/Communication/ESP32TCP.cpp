#include "Core/IO/Communication/ESP32TCP.h"
#include "ESPmDNS.h"
#include <mbedtls/md5.h> 
#include <Core/Logger.h>

namespace Motion::Core::IO {

// Static variable definitions for mDNS management
bool ESP32TCP::_mdnsInitialized = false;
int ESP32TCP::_mdnsRefCount = 0;
SemaphoreHandle_t ESP32TCP::_mdnsMutex = nullptr;

ESP32TCP::ESP32TCP(uint16_t port) 
    : GenericTCP(port), _server(port) {}

ESP32TCP::~ESP32TCP()
{
    StopDiscovery();
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

String getHashedUniqueID()
{
  uint64_t mac = ESP.getEfuseMac(); // 48-bit WIFI @MAC
  unsigned char macBytes[6];
  for (int i = 0; i < 6; i++) macBytes[i] = (mac >> (8*(5-i))) & 0xFF;

  //hash the @MAC 
  unsigned char hash[16];
  mbedtls_md5(macBytes, 6, hash);

  // Format as ESP32-XXXX-XXXX-XXXX
  char id[25];
  snprintf(id, sizeof(id), "ESP32-%02X%02X-%02X%02X-%02X%02X", hash[0], hash[1], hash[2], hash[3], hash[4], hash[5]);
  return String(id);
}

bool ESP32TCP::StartDiscovery(const char* ServiceName)
{
    if (_started == false)
    {
        LOG_WARN("Channel not yet started");
        return false;
    }
    if (ServiceName == nullptr)
    {
        LOG_WARN("Invalid ServiceName");
        return false;
    }

    if (_mdnsMutex == nullptr)
    {
        _mdnsMutex = xSemaphoreCreateMutex();
        if (_mdnsMutex == nullptr)
        {
            return false; // Mutex creation failed
        }
    }

    // Acquire mutex to protect static mDNS state
    if (xSemaphoreTake(_mdnsMutex, portMAX_DELAY) != pdTRUE)
    {
        return false; // Failed to acquire mutex
    }

    // Initialize mDNS if not already done
    if (!_mdnsInitialized)
    {
        LOG_INFO("Initializing mDNS");
        if (!MDNS.begin(getHashedUniqueID().c_str()))
        {
            LOG_WARN("Failed to initialize mDNS");
            xSemaphoreGive(_mdnsMutex);
            return false; // mDNS initialization failed
        }
        _mdnsInitialized = true;
    }
            
    // Add the TCP service to mDNS
    _serviceName = ServiceName;
    if (!MDNS.addService(_serviceName, "tcp", _port))
    {
        LOG_WARN("Failed to add %s service to mDNS", _serviceName);
        xSemaphoreGive(_mdnsMutex);
        return false; // Failed to add service
    }
    LOG_INFO("%s service added to mDNS", _serviceName);

    _mdnsRefCount++;

    // Release mutex
    xSemaphoreGive(_mdnsMutex);

    return true;
}

bool ESP32TCP::StartDiscovery()
{
    if (_serviceName == nullptr)
    {
        return false;
    }

    return StartDiscovery(_serviceName);
}

void ESP32TCP::StopDiscovery()
{
    // Return early if mutex was never created (no discovery was started)
    if (_mdnsMutex == nullptr)
    {
        return;
    }

    // Acquire mutex
    if (xSemaphoreTake(_mdnsMutex, portMAX_DELAY) != pdTRUE)
    {
        return; // Failed to acquire mutex
    }

    if (_mdnsRefCount > 0)
    {
        _mdnsRefCount--;
    }

    // If last discovery being stopped, shut down mDNS
    if (_mdnsRefCount == 0 && _mdnsInitialized)
    {
        MDNS.end();
        _mdnsInitialized = false;
    }

    // Release mutex
    xSemaphoreGive(_mdnsMutex);
}

} // namespace Motion::Core::IO