/**
 * @file ESP32TCPLogging.cpp
 * @brief ESP32 logging over TCP/IP.
 *
 * The logger waits for one TCP client before it starts because ESP32TCP does
 * not buffer messages while no client is connected.
 */

#include <Motion/Core.h>
#include <WiFi.h>

using namespace Motion::Core;
using namespace Motion::Core::IO;

namespace Configuration
{
#ifndef MOTION_WIFI_SSID
#define MOTION_WIFI_SSID "replace-with-wifi-ssid"
#endif

#ifndef MOTION_WIFI_PASSWORD
#define MOTION_WIFI_PASSWORD "replace-with-wifi-password"
#endif

constexpr const char* WifiSsid = MOTION_WIFI_SSID;
constexpr const char* WifiPassword = MOTION_WIFI_PASSWORD;
constexpr uint16_t TcpLogPort = 9501;
} // namespace Configuration

void ConnectWiFi()
{
    WiFi.begin(Configuration::WifiSsid, Configuration::WifiPassword);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
    }
}

void setup()
{
    ConnectWiFi();

    BaseChannelHandle loggerHandle =
        ESP32TCP::Create(Configuration::TcpLogPort);
    if (!loggerHandle || !loggerHandle->Start())
    {
        return;
    }

    while (!loggerHandle->IsConnected())
    {
        delay(100);
    }

    LOG_START(loggerHandle, LogLevel::TRACE);
    LOG_INFO("ESP32 TCP logger connected on port %u", Configuration::TcpLogPort);
}

void loop()
{
    LOG_TRACE("TCP logging heartbeat");
    delay(1000);
}