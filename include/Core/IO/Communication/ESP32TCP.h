#pragma once

#include "Core/IO/Communication/GenericTCP.h"
#include <WiFi.h>

namespace Motion::Core::IO {

class ESP32TCP;
    
/**
 * @brief Defining The ESP32 TCP handle
 */
using ESP32TCPHandle = ChannelPointer(ESP32TCP);

class ESP32TCP : public GenericTCP {
public:
    static ESP32TCPHandle Create(uint16_t port);
    virtual ~ESP32TCP();

    bool Start() override;
    size_t Send(const uint8_t* data, size_t length) override;
    size_t Read(uint8_t* buffer, size_t bufferSize) override;
    bool IsConnected() override;
    void Stop() override;

protected:
    ESP32TCP(uint16_t port);

private:
    WiFiServer _server;
    WiFiClient _client;
};

} // namespace Motion::Core::IO