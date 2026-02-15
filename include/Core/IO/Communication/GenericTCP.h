/**
 * @file GenericTCP.h
 * @brief Header file for the GenericSerial class, providing a generic serial communication interface.
 */

#pragma once

#include "Core/IO/Communication/BaseChannel.h"

namespace Motion::Core::IO {

class GenericTCP : public BaseChannel {
public:
    virtual ~GenericTCP() = default;

protected:
    GenericTCP(uint16_t port) : _port(port), BaseChannel() {};
    uint16_t _port;
};

/**
 * @brief Defining The Generic TCP handle
 */
using GenericTCPHandle = ChannelPointer(GenericTCP);

} // namespace Motion::Core::IO