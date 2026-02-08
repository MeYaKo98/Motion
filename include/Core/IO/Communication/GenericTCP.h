/**
 * @file GenericTCP.h
 * @brief Header file for the GenericSerial class, providing a generic serial communication interface.
 */

#pragma once

#include "Core/IO/Communication/BaseChannel.h"

namespace Motion::Core::IO {

class GenericTCP : public BaseChannel {
public:

    GenericTCP(uint16_t port) : _port(port), BaseChannel() {};

    virtual ~GenericTCP() = default;

protected:
    uint16_t _port;
};

} // namespace Motion::Core::IO