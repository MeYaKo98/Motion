#include "Motion/Link/Link.h"
#include "Motion/Link/LinkTypeDef.h"
#include "Motion/Core/Robot/Navigation/DifferentialDriveNavigation.h"


namespace Motion {

template <>
void Link::AttachCallbacks<Motion::Core::Robot::GenericDifferentialDriveNavigation>(Motion::Core::Robot::GenericDifferentialDriveNavigationHandle& object)
{   
    AttachFastCallback(
        (uint16_t)Motion::Bridge::Command::GetDrive,
        [&object](Motion::Bridge::Request* req, Motion::Bridge::Response* res)
        {
            Motion::Bridge::RobotDrive drive = Motion::Bridge::RobotDrive::DifferentialDrive;
            res->status = Motion::Bridge::Status::OK;
            res->length = sizeof(Motion::Bridge::RobotDrive);
            memcpy(res->data, &drive, sizeof(drive));
        }
    );

    AttachCallbacks<Motion::Core::Robot::BaseNavigation>((Motion::Core::Robot::BaseNavigationHandle&)object);
    AttachCallbacks<Motion::Core::Robot::BaseOdometry>((Motion::Core::Robot::BaseOdometryHandle&)object->_odometryHandle);
}

template <>
void Link::AttachCallbacks<Motion::Core::Robot::DifferentialDriveNavigation>(Motion::Core::Robot::DifferentialDriveNavigationHandle& object)
{
    AttachCallbacks<Motion::Core::Robot::GenericDifferentialDriveNavigation>((Motion::Core::Robot::GenericDifferentialDriveNavigationHandle&)object);
}

} // namespace Motion