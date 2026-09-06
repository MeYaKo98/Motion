#include "Motion/Link/Link.h"
#include "Motion/Core/Robot/Odom/BaseOdometry.h"
#include "Motion/Core/Robot/Navigation/BaseNavigation.h"


namespace Motion {

/** @cond INTERNAL */

template <>
void Link::AttachCallbacks<Motion::Core::Robot::BaseOdometry>(Motion::Core::Robot::BaseOdometryHandle& object)
{
    Motion::Link::AttachFastCallback(
        (uint16_t)Motion::Bridge::Command::GetPosition,
        [&object](Motion::Bridge::Request* req, Motion::Bridge::Response* res)
        {
            Motion::Core::Robot::Position pos = object->GetPosition();
            Motion::Bridge::Position out = {pos.x, pos.y, pos.theta};
            res->status = Motion::Bridge::Status::OK;
            res->length = sizeof(out);
            memcpy(res->data, &out, sizeof(out));
        }
    );
}

template <>
void Link::AttachCallbacks<Motion::Core::Robot::BaseNavigation>(Motion::Core::Robot::BaseNavigationHandle& object)
{
    Motion::Link::AttachSlowCallback(
        (uint16_t)Motion::Bridge::Command::MoveTo,
        [&object](Motion::Bridge::Request* req)
        {
            if (req->length < sizeof(Bridge::Point))
            {
                LOG_WARN("Invalid request length");
                return;
            }
            Bridge::Point point;
            memcpy(&point, req->data, sizeof(point));
            object->MoveTo(point.x, point.y);
        }
    );

    Motion::Link::AttachSlowCallback(
        (uint16_t)Motion::Bridge::Command::Orient,
        [&object](Motion::Bridge::Request* req)
        {
            if (req->length < sizeof(float))
            {
                LOG_WARN("Invalid request length");
                return;
            }
            float orientation;
            memcpy(&orientation, req->data, sizeof(orientation));
            object->Orient(orientation);
        }
    );

    Motion::Link::AttachSlowCallback(
        (uint16_t)Motion::Bridge::Command::Move,
        [&object](Motion::Bridge::Request* req)
        {
            if (req->length < sizeof(float))
            {
                LOG_WARN("Invalid request length");
                return;
            }
            float distance;
            memcpy(&distance, req->data, sizeof(distance));
            object->Move(distance);
        }
    );

    Motion::Link::AttachSlowCallback(
        (uint16_t)Motion::Bridge::Command::Turn,
        [&object](Motion::Bridge::Request* req)
        {
            if (req->length < sizeof(float))
            {
                LOG_WARN("Invalid request length");
                return;
            }
            float theta;
            memcpy(&theta, req->data, sizeof(theta));
            object->Turn(theta);
        }
    );
}

/** @endcond */

}