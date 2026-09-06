/**
 * @file MotionLinkCustomCommandTemplate.cpp
 * @brief Application-defined Motion Link command template.
 */

#include "Motion/Link/Link.h"

namespace Application
{

constexpr uint16_t GetTelemetryCommand =
    static_cast<uint16_t>(Motion::Bridge::Command::UserCommandStart);

void AttachTelemetryCommand()
{
    Motion::Link::AttachFastCallback(
        GetTelemetryCommand,
        [](Motion::Bridge::Request* request,
           Motion::Bridge::Response* response)
        {
            if (request->length != 0)
            {
                response->status = Motion::Bridge::Status::InvalidRequestLength;
                response->length = 0;
                return;
            }

            // Fill a small response payload with short, non-blocking work.
            response->status = Motion::Bridge::Status::OK;
            response->length = 0;
        });
}

void AttachSlowOperation(uint16_t command)
{
    Motion::Link::AttachSlowCallback(
        command,
        [](Motion::Bridge::Request* request)
        {
            // Validate request->length before reading request->data.
            // Long-running work belongs here, outside the receive loop.
            (void)request;
        });
}

} // namespace Application
