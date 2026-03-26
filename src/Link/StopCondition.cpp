#include "Motion/Link/Link.h"
#include "Motion/Core/Robot/StopCondition/ExternalStopCondition.h"


namespace Motion {

template <>
void Link::AttachCallbacks<Motion::Core::Robot::ExternalStopCondition>(Motion::Core::Robot::ExternalStopConditionHandle& object)
{
    Motion::Link::AttachFastCallback(
        (uint16_t)Motion::Bridge::Command::Stop,
        [&object](Motion::Bridge::Request* req, Motion::Bridge::Response* res)
        {
            object->SetFlag(true);
            res->status = Motion::Bridge::Status::OK;
            res->length = 0;
        }
    );
}

}