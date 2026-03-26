#include "Motion/Core/IO/Communication/BaseChannel.h"
#include "Motion/Core/IO/Communication/GenericTCP.h"
#include "Motion/Core/IO/Communication/GenericSerial.h"
#include "Motion/Link/LinkTypeDef.h"
#include "Motion/Core/Logger.h"

#include "freertos/FreeRTOS.h"

#include <utility>
#include <unordered_map>
#include <functional>

#define MotionLinkTCPPort 9500
#define MotionLinkMdnsService "MotionLink"

#define COMMAND_MAX_LENGTH            64
#define COMMAND_QUEUE_SIZE            8 

namespace Motion {

class Link {
public:

    template <typename T>
    static bool StartTCP ()
    {
        static_assert(std::is_base_of<Motion::Core::IO::GenericTCP, T>::value, "T must derive from GenericTCP");
        if (GetInstance()._channel)
        {
            LOG_WARN("MotionLink already started");
            return false;
        }

        //Create the Channel
        Motion::Core::IO::GenericTCPHandle tcpHandle = T::Create(MotionLinkTCPPort);
        if (!tcpHandle)
        {
            LOG_WARN("Failed to create TCP Channel");
            return false;
        }

        //Start the socket
        if (!tcpHandle->Start())
        {
            LOG_WARN("Failed to start TCP Channel");
            return false;
        }

        //Broadcast via mdns
        if (!tcpHandle->StartDiscovery(MotionLinkMdnsService))
        {
            LOG_WARN("Failed to start TCP Discovery");
            return false;
        }

        GetInstance()._channel = tcpHandle;
        AttachCallbacks<Link>(&GetInstance());

        return true;
    }

    template <typename T, typename... Args>
    static bool StartSerial (Args&&... args)
    {
        static_assert(std::is_base_of<Motion::Core::IO::GenericSerial, T>::value, "T must derive from GenericSerial");
        if (GetInstance()._channel)
        {
            LOG_WARN("MotionLink already started");
            return false;
        }

        //Create the Channel
        Motion::Core::IO::GenericSerialHandle serialHandle = T::Create(std::forward<Args>(args)...);
        if (!serialHandle)
        {
            LOG_WARN("Failed to create Serial Channel");
            return false;
        }

        //Start the channel
        if (!serialHandle->Start())
        {
            LOG_WARN("Failed to start Serial Channel");
            return false;
        }

        GetInstance()._channel = serialHandle;
        AttachCallbacks<Link>(&GetInstance());

        return true;
    }

    using FastCallbackPtr = std::function<void(Motion::Bridge::Request*, Motion::Bridge::Response*)>;
    static void AttachFastCallback(uint16_t command, FastCallbackPtr callback);

    using SlowCallbackPtr = std::function<void(Motion::Bridge::Request*)>;
    static void AttachSlowCallback(uint16_t command, SlowCallbackPtr callback);

    static bool DetachCallback(uint16_t command);

    template <typename T>
    static void AttachCallbacks(T* object);

    template <typename T>
    static void AttachCallbacks(std::unique_ptr<T>& object);

    template <typename T>
    static void AttachCallbacks(std::shared_ptr<T>& object);

    static void Spin();

    Link(Link const&) = delete;
    void operator=(Link const&) = delete;

private:


    Link();

    static Link& GetInstance();
    
    static void SlowCommandTask(void* pvParameters);

    Motion::Core::IO::BaseChannelHandle _channel;

    std::unordered_map<uint16_t, FastCallbackPtr> _fastCallbackRegistry;
    std::unordered_map<uint16_t, SlowCallbackPtr> _slowCallbackRegistry;

    QueueHandle_t _slowCommandQueue;
};

} // namespace Motion::Link