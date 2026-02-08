#pragma once

#include "Core/IO/Communication/GenericTCP.h"
#include <WiFi.h>

namespace Motion::Core::IO {

class ESP32TCP : public GenericTCP {
public:
    ESP32TCP(uint16_t port);
    virtual ~ESP32TCP();

    bool Start() override;
    size_t Send(const uint8_t* data, size_t length) override;
    size_t Read(uint8_t* buffer, size_t bufferSize) override;
    bool IsConnected() override;
    void Stop() override;

private:
    WiFiServer _server;
    WiFiClient _client;
};

} // namespace Motion::Core::IO