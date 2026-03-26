#include "Motion/Link/Link.h"

namespace Motion {

Link::Link() : _channel(nullptr)
{
    _slowCommandQueue = xQueueCreate(COMMAND_QUEUE_SIZE, COMMAND_MAX_LENGTH);
    if (!_slowCommandQueue)
    {
        throw std::runtime_error("Failed to create Slow Command Queue");
    }
}

Link& Link::GetInstance() 
{
    static Link instance;
    return instance;
} 

void Link::AttachFastCallback(uint16_t command, FastCallbackPtr callback)
{
    if (!callback)
    {
        LOG_WARN("Callback is nullptr");
        return;
    }
    if (Link::DetachCallback(command))
    {
        LOG_WARN("Callback %d overwritten", command);
    }
    GetInstance()._fastCallbackRegistry[command] = callback;
}

void Link::AttachSlowCallback(uint16_t command, SlowCallbackPtr callback)
{
    if (!callback)
    {
        LOG_WARN("Callback is nullptr");
        return;
    }
    if (Link::DetachCallback(command))
    {
        LOG_WARN("Callback %d overwritten", command);
    }
    GetInstance()._slowCallbackRegistry[command] = callback;
}

bool Link::DetachCallback(uint16_t command)
{
    if ((GetInstance()._slowCallbackRegistry.erase(command) > 0) || (GetInstance()._fastCallbackRegistry.erase(command) > 0))
        return true;
    return false;
}

template<>
void Link::AttachCallbacks<Link>(Link* object)
{
    AttachFastCallback(
        (uint16_t)Motion::Bridge::Command::GetStatus,
        [](Motion::Bridge::Request* req, Motion::Bridge::Response* res)
        {
            res->status = Motion::Bridge::Status::OK;
            res->length = 0;
        }
    );

    AttachFastCallback(
        (uint16_t)Motion::Bridge::Command::BridgeVersion,
        [](Motion::Bridge::Request* req, Motion::Bridge::Response* res)
        {
            res->status = Motion::Bridge::Status::OK;
            res->length = strlen(Motion::Bridge::Version);
            memcpy(res->data, Motion::Bridge::Version, res->length);
        }
    );

    AttachFastCallback(
        (uint16_t)Motion::Bridge::Command::QueueStatus,
        [](Motion::Bridge::Request* req, Motion::Bridge::Response* res)
        { 
            res->length = 0;
            if (uxQueueMessagesWaiting(GetInstance()._slowCommandQueue) == 0)
                res->status = Motion::Bridge::Status::OK;
            else
                res->status = Motion::Bridge::Status::QueueFull;
        }
    );

    AttachFastCallback(
        (uint16_t)Motion::Bridge::Command::EmptyQueue,
        [](Motion::Bridge::Request* req, Motion::Bridge::Response* res)
        { 
            res->length = 0;
            if (xQueueReset(GetInstance()._slowCommandQueue) == pdPASS)
                res->status = Motion::Bridge::Status::OK;
            else
                res->status = Motion::Bridge::Status::UnknownError;
        }
    );
}

void Link::Spin()
{
    if (!GetInstance()._channel)
    {
        LOG_WARN("MotionLink not started");
        return;
    }

    if (!xTaskCreate(Link::SlowCommandTask, "SlowCommandTask", 8192, NULL, 1, NULL))
    {
        LOG_WARN("Failed to create Slow Command Task");
        return;
    }

    uint8_t reqBuffer[COMMAND_MAX_LENGTH];
    uint8_t resBuffer[COMMAND_MAX_LENGTH];
    int Len = 0;
    
    Motion::Bridge::Request* req = (Motion::Bridge::Request*)reqBuffer;
    Motion::Bridge::Response* res = (Motion::Bridge::Response*)resBuffer;

    std::unordered_map<uint16_t, FastCallbackPtr>::iterator itFast;

    while (true)
    {
        if ((Len = GetInstance()._channel->Read(reqBuffer, sizeof(reqBuffer)))!=0)
        {
            if (sizeof(req->command)+sizeof(req->length)+req->length > Len)
            {
                LOG_WARN("Invalid request length");
                res->status = Motion::Bridge::Status::InvalidRequestLength;
                res->length = 0;
            }
            else if ((itFast = GetInstance()._fastCallbackRegistry.find(req->command)) != GetInstance()._fastCallbackRegistry.end())
            {
                itFast->second(req, res);
            }
            else if (GetInstance()._slowCallbackRegistry.find(req->command) != GetInstance()._slowCallbackRegistry.end())
            {
                if (xQueueSend(GetInstance()._slowCommandQueue, reqBuffer, 0) == pdPASS)
                {
                    res->status = Motion::Bridge::Status::OK;
                }
                else
                {
                    res->status = Motion::Bridge::Status::QueueFull;
                }
                res->length = 0;
            }
            else
            {
                LOG_WARN("Unknown command");
                res->status = Motion::Bridge::Status::UnkownCommand;
                res->length = 0;
            }
            GetInstance()._channel->Send(resBuffer, sizeof(res->status)+sizeof(res->length)+res->length);
        }
    }
}

void Link::SlowCommandTask(void* pvParameters)
{
    uint8_t buffer[COMMAND_MAX_LENGTH];
    Motion::Bridge::Request* msg = (Motion::Bridge::Request*) buffer;
    while (true)
    {
        if (xQueueReceive(GetInstance()._slowCommandQueue, buffer, portMAX_DELAY) == pdPASS)
        {
            GetInstance()._slowCallbackRegistry.at(msg->command)(msg);
            LOG_TRACE("Task completed");
        }
    }
}

}