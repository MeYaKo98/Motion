/**
 * @file MotionLinkTcpTemplate.cpp
 * @brief TCP Motion Link server composition template.
 *
 * Replace the placeholder channel and robot types with implementations from
 * the target application. This file is a documentation template, not a
 * board-complete executable.
 */

#include "Motion/Link/Link.h"

// Replace these names with platform and robot implementations.
using TcpChannel = MyTcpChannel;
using RobotNavigation = MyDifferentialDriveNavigation;

void StartMotionLinkTcp()
{
    auto navigation = RobotNavigation::Create(/* robot configuration */);
    navigation->Start();

    if (!Motion::Link::StartTCP<TcpChannel>())
    {
        // Report the failure and keep the robot in a safe state.
        return;
    }

    Motion::Link::AttachCallbacks(navigation);

    // Spin owns this task for the rest of the communication service lifetime.
    Motion::Link::Spin();
}
