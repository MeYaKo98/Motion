/**
 * @file Link.h
 * @brief Motion Link middleware for exposing Motion Framework services remotely.
 */

#include "Motion/Core/IO/Communication/BaseChannel.h"
#include "Motion/Core/IO/Communication/GenericTCP.h"
#include "Motion/Core/IO/Communication/GenericSerial.h"
#include "Motion/Link/LinkTypeDef.h"
#include "Motion/Core/Logger.h"

#include "freertos/FreeRTOS.h"

#include <utility>
#include <unordered_map>
#include <functional>

/** @brief TCP listening port used by the Motion Link server. */
#define MotionLinkTCPPort 9500

/** @brief mDNS service name advertised by the TCP Link server. */
#define MotionLinkMdnsService "MotionLink"

/** @brief Maximum request or response buffer size in bytes. */
#define COMMAND_MAX_LENGTH            64

/** @brief Maximum number of slow requests waiting in the FreeRTOS queue. */
#define COMMAND_QUEUE_SIZE            8

namespace Motion {

/**
 * @class Link
 * @brief Singleton-style transport and callback dispatcher for Motion Link.
 * @details
 * `Motion::Link` is the middleware layer between a Motion Framework robot and a
 * Motion Link client. It reads packed `Motion::Bridge::Request` messages from a
 * framework communication channel, dispatches registered callbacks, and writes
 * a packed `Motion::Bridge::Response` for each request.
 *
 * **Link Architecture:**
 * - Transport Adapter: Uses a `BaseChannel` implementation over TCP or serial
 * - Command Registry: Maps numeric bridge commands to fast or slow callbacks
 * - Request Dispatcher: Validates request lengths and selects a callback
 * - Slow Command Queue: Buffers up to `COMMAND_QUEUE_SIZE` slow requests
 * - Worker Task: Executes slow callbacks outside the receive loop
 * - Response Encoder: Sends the callback result back through the channel
 *
 * **Task Lifecycle:**
 * 1. `StartTCP()` or `StartSerial()` creates and starts the transport
 * 2. Built-in status, version, and queue callbacks are registered
 * 3. Application callbacks are attached with `AttachCallbacks<T>()` or the
 *    explicit callback registration methods
 * 4. `Spin()` creates the slow-command worker and enters the receive loop
 * 5. Each request is dispatched and acknowledged with one response
 *
 * **Middleware Requirements:**
 * A complete deployment requires a compatible transport implementation, the
 * FreeRTOS task and queue APIs, and a client that uses the same packed bridge
 * definitions. Link is intended for embedded targets where the Motion Core
 * communication and FreeRTOS layers are already available.
 *
 * A target must provide:
 * - a `Motion::Core::IO::GenericTCP` or `Motion::Core::IO::GenericSerial`
 *   implementation with a compatible `Create()` factory;
 * - a working `Motion::Core::IO::BaseChannel` implementation whose `Start()`,
 *   `Read()`, and `Send()` methods are usable by the link;
 * - FreeRTOS task and queue support, because slow callbacks are processed by a
 *   dedicated task; and
 * - the Motion Link wire definitions from `LinkTypeDef.h` on both peers.
 *
 * **Communication Protocol:**
 * Each request is encoded as a packed `uint16_t command`, a packed
 * `uint16_t length`, and `length` payload bytes. The response is encoded as a
 * packed `Motion::Bridge::Status`, a packed `uint16_t length`, and `length`
 * payload bytes. The channel is treated as a byte stream; the channel or its
 * peer must preserve message boundaries because Link does not add framing or
 * checksum bytes.
 *
 * **Built-in Commands:**
 * - `GetStatus`: returns `Status::OK` with no payload.
 * - `BridgeVersion`: returns the bytes of `Version`.
 * - `QueueStatus`: reports `Status::OK` when the slow-command queue is empty,
 *   otherwise `Status::QueueFull`.
 * - `ClearQueue`: resets the slow-command queue.
 *
 * Robot service callbacks are added with `AttachCallbacks<T>()`. The available
 * framework mappings include drive type, position, navigation commands, and
 * the external stop command. Applications may register commands beginning at
 * `Motion::Bridge::Command::UserCommandStart`.
 *
 * **Typical Usage:**
 * @code
 * auto navigation = DifferentialDriveNavigation::Create(...);
 * navigation->Start();
 *
 * if (Motion::Link::StartTCP<MyTcpChannel>())
 * {
 *     Motion::Link::AttachCallbacks(navigation);
 *     Motion::Link::Spin();
 * }
 * @endcode
 *
 * **Serial Usage:**
 * @code
 * if (Motion::Link::StartSerial<MySerialChannel>(serialPort, 115200))
 * {
 *     Motion::Link::Spin();
 * }
 * @endcode
 *
 * @note `Spin()` normally does not return. The application should run it in the
 *       task or thread that owns the communication service.
 *
 * @note The current implementation treats one `Read()` result as one complete
 *       request. A channel implementation must preserve request boundaries or
 *       provide equivalent buffering before returning from `Read()`.
 *
 * @warning Do not start both TCP and serial transports on the same Link
 *          instance. Link owns one active channel.
 *
 * @warning Do not register callbacks that capture stack objects which will be
 *          destroyed while Link is running.
 *
 * @warning `Spin()` is an infinite processing loop. Run it in the task or
 *          thread intended to own the link, and do not call it before a
 *          successful `StartTCP()` or `StartSerial()` call.
 *
 * The class owns one active channel and callback registries for fast and slow
 * commands. Fast callbacks run in the receive loop and must be short and
 * non-blocking. Slow callbacks are copied into an eight-entry FreeRTOS queue
 * and run asynchronously by `SlowCommandTask()`.
 *
 * A command can have only one registered callback. Registering a callback for
 * an existing command replaces the previous callback. `DetachCallback()` removes
 * either callback kind for the command.
 */
class Link {
public:

    /**
     * @brief Starts a TCP Motion Link transport.
    * @details Creates a TCP channel on port `MotionLinkTCPPort`, starts the
    *          socket, advertises the mDNS service, and installs the built-in
    *          Link callbacks.
     * @tparam T Concrete channel derived from `GenericTCP`.
    * @pre The FreeRTOS scheduler and network stack are initialized.
     * @return `true` when the channel, TCP socket, discovery advertisement, and
     *         built-in callbacks are initialized; otherwise `false`.
    * @post A successful call leaves one active channel owned by Link.
    * @warning Calling this method a second time returns `false` and does not
    *          replace the active channel.
     * @note The TCP listener uses port `MotionLinkTCPPort` and advertises
     *       `MotionLinkMdnsService`.
     */
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

    /**
     * @brief Starts a serial Motion Link transport.
    * @details Creates the serial channel by forwarding all arguments to
    *          `T::Create()`, starts it, and installs the built-in callbacks.
     * @tparam T Concrete channel derived from `GenericSerial`.
     * @param args Arguments forwarded to `T::Create()`.
    * @pre The serial hardware and its driver are initialized.
     * @return `true` when the channel and built-in callbacks are initialized.
    * @post A successful call leaves one active channel owned by Link.
    * @warning Calling this method a second time returns `false` and does not
    *          replace the active channel.
     */
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

    /**
     * @brief Callback signature for immediate request processing.
     * @details
     * Fast callbacks execute in the receive loop and must fill `res->status`
     * and `res->length` before returning. They may write response bytes to
     * `res->data`. The response is sent immediately after the callback returns.
    *
    * @param[in] req Pointer to the request buffer. The callback must not retain
    *                 this pointer after it returns.
    * @param[out] res Pointer to the response buffer. Set both status and length
    *                  even when the response has no payload.
    * @note Fast callbacks execute in the Link receive task. Blocking, waiting
    *       for motion completion, or performing long I/O delays all other
    *       incoming requests.
     */
    using FastCallbackPtr = std::function<void(Motion::Bridge::Request*, Motion::Bridge::Response*)>;
    /**
    * @brief Registers or replaces a fast callback for a numeric command.
    * @param command Numeric value of the `Motion::Bridge::Command`.
    * @param callback Callback invoked immediately when the command is received.
    * @pre `callback` must be non-null.
    * @post Any existing callback for the command is replaced.
    */
    static void AttachFastCallback(uint16_t command, FastCallbackPtr callback);

    /**
     * @brief Callback signature for queued request processing.
     * @details
     * Slow callbacks execute asynchronously in the link's FreeRTOS worker task.
     * Their acknowledgement only confirms that the request entered the queue;
     * it does not mean the operation has finished. Slow callbacks have no
     * response buffer.
    *
    * @param[in] req Pointer to a copied request in the worker task buffer.
    * @note The callback should validate `req->length` before reading payload
    *       bytes and should return promptly enough for the queue to drain.
     */
    using SlowCallbackPtr = std::function<void(Motion::Bridge::Request*)>;
    /**
    * @brief Registers or replaces a slow callback for a numeric command.
    * @param command Numeric value of the `Motion::Bridge::Command`.
    * @param callback Callback placed behind the slow-command queue.
    * @pre `callback` must be non-null.
    * @post Any existing callback for the command is replaced.
    * @note The client receives `Status::OK` when the request is queued, not
    *       when the callback has completed.
    */
    static void AttachSlowCallback(uint16_t command, SlowCallbackPtr callback);

    /**
     * @brief Removes the callback registered for @p command, if present.
     * @param command Numeric command whose callback should be removed.
     * @return `true` if a callback was removed; `false` if none was registered.
     * @note Removing a callback does not cancel a request already in the queue.
     */
    static bool DetachCallback(uint16_t command);

    /**
     * @brief Attaches the framework callbacks associated with an object type.
     * @tparam T A type with a Motion Link callback specialization.
     * @param object Object whose services are exposed to the bridge.
    * @pre The object must remain alive while its callbacks are registered.
    * @post Commands supported by the specialization are added to the registry.
     * @note Supported framework specializations include robot navigation,
     *       odometry, drive, and external stop-condition types. Custom types
     *       may provide a matching specialization in the application.
     */
    template <typename T>
    static void AttachCallbacks(T* object);

    /** @copydoc AttachCallbacks(T*) */
    template <typename T>
    static void AttachCallbacks(std::unique_ptr<T>& object);

    /** @copydoc AttachCallbacks(T*) */
    template <typename T>
    static void AttachCallbacks(std::shared_ptr<T>& object);

    /**
     * @brief Runs the receive, dispatch, and response loop.
     * @details Starts the worker task for slow callbacks, then continuously
     *          reads requests from the active channel. Unknown commands return
     *          `Status::UnkownCommand`; malformed requests return
     *          `Status::InvalidRequestLength`; a full slow queue returns
     *          `Status::QueueFull`.
     * @pre A successful `StartTCP()` or `StartSerial()` call.
    * @post The method continuously services the active channel until its task
    *       is terminated or the system is reset.
    * @note A malformed request is rejected when its declared payload extends
    *       beyond the bytes returned by the channel.
    * @warning This method blocks forever in normal operation and must not be
    *          called from an interrupt service routine.
     */
    static void Spin();

    Link(Link const&) = delete;
    void operator=(Link const&) = delete;

private:

    /** @brief Constructs the singleton Link state and its slow-command queue. */
    Link();

    /** @brief Returns the process-wide Link instance. */
    static Link& GetInstance();

    /**
     * @brief FreeRTOS entry point that executes queued slow callbacks.
     * @param pvParameters Reserved FreeRTOS task parameter; currently unused.
     */
    static void SlowCommandTask(void* pvParameters);

    /** @brief The single active TCP or serial transport. */
    Motion::Core::IO::BaseChannelHandle _channel;

    /** @brief Registry of callbacks executed immediately in the receive loop. */
    std::unordered_map<uint16_t, FastCallbackPtr> _fastCallbackRegistry;

    /** @brief Registry of callbacks dispatched through the slow-command task. */
    std::unordered_map<uint16_t, SlowCallbackPtr> _slowCallbackRegistry;

    /** @brief FreeRTOS queue storing copied slow-command request frames. */
    QueueHandle_t _slowCommandQueue;
};

} // namespace Motion::Link