/**
 * @file GenericSerial.h
 * @brief Header file for the GenericSerial class, providing a generic serial communication interface.
 */

#pragma once

#include "Core/IO/Communication/BaseChannel.h"

namespace Motion::Core::IO {

/**
 * @class GenericSerial
 * @brief A generic implementation of a serial communication channel.
 * @details This class extends `BaseChannel` to represent a serial communication endpoint.
 *          It is intended to be used for standard serial I/O operations.
 * @see BaseChannel
 */
class GenericSerial : public BaseChannel {
public:
    /**
     * @brief Default constructor.
     * @details Initializes a new instance of the `GenericSerial` class.
     * @param baudRate The baud rate for communication (e.g., 9600, 115200).
     */
    GenericSerial(uint32_t baudRate) : _baudRate(baudRate), BaseChannel() {};

    /**
     * @brief Virtual destructor.
     * @details Destroys the `GenericSerial` object.
     *          The default implementation ensures that derived classes are properly destroyed.
     */
    virtual ~GenericSerial() = default;

protected:
    uint32_t _baudRate;
};

} // namespace Motion::Core::IO