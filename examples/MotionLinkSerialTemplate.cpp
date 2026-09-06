/**
 * @file MotionLinkSerialTemplate.cpp
 * @brief Serial Motion Link server composition template.
 *
 * Replace the placeholder serial channel with a GenericSerial implementation
 * for the target board and provide the constructor arguments required by its
 * Create() factory.
 */

#include "Motion/Link/Link.h"

using SerialChannel = MySerialChannel;

void StartMotionLinkSerial()
{
    // StartSerial forwards these arguments to SerialChannel::Create().
    if (!Motion::Link::StartSerial<SerialChannel>(SerialPort, 115200))
    {
        return;
    }

    Motion::Link::Spin();
}
