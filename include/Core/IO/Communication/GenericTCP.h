/**
 * @file GenericTCP.h
 * @brief Header file for the GenericTCP class, providing a generic TCP communication interface.
 * @details Extends BaseChannel to represent a TCP/IP socket communication endpoint.
 */

#pragma once

#include "Core/IO/Communication/BaseChannel.h"

namespace Motion::Core::IO {

/**
 * @class GenericTCP
 * @brief A generic implementation of a TCP/IP communication channel.
 * @details This class extends `BaseChannel` to provide a standardized abstraction for
 *          TCP socket communication over Ethernet or WiFi networks. It is a base class
 *          for platform-specific implementations (e.g., ESP32TCP) that handle
 *          the actual socket and network interface details.
 *
 *          **TCP Communication Context:**
 *          TCP (Transmission Control Protocol) provides reliable, connection-oriented communication:
 *          - Stream-based: Data is delivered as a stream of bytes without message boundaries
 *          - Ordered: Bytes arrive in the same order they were sent (guaranteed by TCP layer)
 *          - Reliable: Lost or corrupted packets are automatically retransmitted
 *          - Connection-oriented: Must establish a connection before data exchange
 *          - Flow control: TCP automatically adjusts transmission rate based on receiver capacity
 *
 *          **Use Cases in Robotics:**
 *          - Remote teleoperation over WiFi or Ethernet
 *          - Wireless logging and diagnostics
 *          - Inter-robot communication on a local network
 *          - Communication with a central control station or cloud service
 *          - Command streaming from a planning computer
 *
 *          **TCP vs. Serial:**
 *          | Feature | Serial | TCP |
 *          |---------|--------|-----|
 *          | Speed | 115.2 kbps typical | 1-1000 Mbps |
 *          | Distance | < 20m (limited by cable) | 100m+ (router dependent) |
 *          | Reliability | Basic parity check | Full error detection/correction |
 *          | Latency | 1-10 ms | 10-100 ms (depends on network) |
 *          | Complexity | Simple | Higher (network stack) |
 *
 * @note **Network Configuration:** TCP communication requires network setup:
 *       - IP address assignment (DHCP or static)
 *       - Port number selection (must avoid system-reserved ports < 1024)
 *       - Firewall configuration (some networks block custom ports)
 *
 * @note **Blocking Behavior:** TCP operations (especially connection establishment) may block.
 *       For real-time applications, consider using non-blocking sockets or timeouts.
 *
 * @see BaseChannel for the general communication channel interface
 * @see GenericSerial for serial (UART) communication
 */
class GenericTCP : public BaseChannel {
public:
    /**
     * @brief Virtual destructor.
     * @details Ensures proper polymorphic destruction of derived instances.
     *          The default implementation is typically sufficient unless a derived class
     *          manages additional resources.
     *
     * @note Default implementation `= default` uses the compiler-generated destructor,
     *       which is appropriate for simple classes without explicit heap allocations.
     */
    virtual ~GenericTCP() = default;

    /**
     * @brief Starts service discovery with a specified service name.
     * @details Enables network service discovery (mDNS/Bonjour) for the TCP channel.
     *          This allows remote devices to automatically discover this service on the local network
     *          without needing to know the IP address in advance. The service is published with the
     *          specified name and can be discovered by other devices using standard mDNS queries.
     *
     *          **Service Discovery Mechanism:**
     *          - Registers a DNS-SD service under the given name
     *          - Responds to multicast DNS queries (224.0.0.251:5353)
     *          - Enables automatic discovery tools (Bonjour, Avahi, iOS/Android discovery)
     *          - Service remains discoverable as long as the device is running
     *
     *          **Typical Usage in Robotics:**
     *          - Robot advertises as "_robot_service._tcp.local" (humanoid-robot._robot_service._tcp.local)
     *          - Remote control station discovers without hardcoded IP addresses
     *          - Multiple robot instances can be discovered by service name
     *          - Simplifies network configuration for field deployments
     *
     * @param ServiceName The human-readable name for the service (e.g., "robot-1", "arm-controller").
     *                    - Maximum length: typically 63 characters (DNS label limit)
     *                    - Valid characters: alphanumeric, hyphens, underscores (some restrictions apply)
     *                    - Examples: "robot-arm", "drone-01", "motion-controller"
     *                    - The pointer is stored internally; the caller must ensure this memory remains valid
     *                      for the lifetime of the discovery.
     *
     * @return true if service discovery started successfully.
     *         - Successfully registered with the underlying mDNS/Bonjour service
     *         - Service is now discoverable on the local network
     *
     * @return false if discovery failed.
     *         - mDNS service not available (e.g., not initialized on the platform)
     *         - Service name is invalid or too long
     *         - Network not ready
     *         - Service already registered (call StopDiscovery() first)
     *
     * @post On success:
     *       - Service name is stored internally (pointer stored, not copied)
     *       - mDNS announces the service on the network
     *       - Remote devices can discover this service by querying mDNS
     *       - StopDiscovery() must be called to unregister
     *
     * @post On failure:
     *       - No state changes; safe to retry
     *       - Service is not discoverable
     *
     * @note **Memory Management:** The ServiceName pointer is stored directly, not copied.
     *       The caller must ensure the string remains valid. Typically use string literals
     *       or statically-allocated strings:
     *       @code
     *       tcp->StartDiscovery("my-robot");       // OK (string literal)
     *       const char* name = "robot-01";
     *       tcp->StartDiscovery(name);             // OK if name persists
     *       char buffer[] = "temp";
     *       tcp->StartDiscovery(buffer);           // RISKY: buffer may be deallocated
     *       @endcode
     *
     * @note **Concurrent Discovery:** A service can only be registered once.
     *       Calling StartDiscovery() multiple times without StopDiscovery() in between
     *       may fail or update the existing service (implementation-dependent).
     *
     * @note **Platform-Specific:** mDNS availability depends on the platform:
     *       - ESP32: Full mDNS support via mdns.h
     *       - ARM Cortex: Platform-dependent (check SDK)
     *       - Platforms without mDNS return false
     *
     * @note **Thread-Safety:** The implementation must handle thread-safe service registration.
     *       For FreeRTOS platforms, use internal mutexes to protect static mDNS state.
     *
     * @warning **Service Name Stability:** Once StartDiscovery() succeeds, the service name
     *          should not change frequently. Changing the service name requires StopDiscovery()
     *          followed by a new StartDiscovery() call, which may take a few seconds to propagate.
     *
     * @warning **Network Dependency:** mDNS requires UDP port 5353 on the local network.
     *          Firewall rules, network segmentation, or WiFi isolation may prevent discovery.
     *          Ensure the network allows multicast traffic.
     *
     * @see StartDiscovery() to re-enable discovery with an existing service name
     * @see StopDiscovery() to unregister the service
     */
    virtual bool StartDiscovery(const char* ServiceName) = 0;

    /**
     * @brief Re-enables service discovery with a previously configured service name.
     * @details Resumes mDNS service discovery using the service name provided in a previous
     *          StartDiscovery(const char*) call. This is useful after temporarily stopping discovery
     *          (via StopDiscovery()) without needing to re-specify the service name.
     *
     *          **Use Cases:**
     *          - Pause network discovery during system reconfiguration
     *          - Temporarily disable during high-load periods to reduce network overhead
     *          - Restart discovery after a network interruption (reconnection flow)
     *
     * @return true if discovery re-enabled successfully.
     *         - Service is now registered and discoverable again
     *         - Uses the previously stored service name
     *
     * @return false if re-enabling failed.
     *         - No previous service name was stored (StartDiscovery(const char*) never called)
     *         - mDNS service not available
     *         - Network not ready
     *
     * @pre A prior call to StartDiscovery(const char* ServiceName) must have succeeded.
     *      The service name must still be valid (pointer still references valid memory).
     *
     * @post On success:
     *       - mDNS service re-announced on the network
     *       - Service becomes discoverable again with the original service name
     *
     * @note **Service Name Reuse:** This function uses the exact same service name pointer
     *       that was provided to the previous StartDiscovery(const char*) call.
     *       If the pointer is no longer valid, behavior is undefined.
     *
     * @note **Idempotency:** Calling StartDiscovery() twice in succession (without StopDiscovery())
     *       is typically safe; the second call may update or confirm the existing registration.
     *
     * @note **Zero-Configuration:** This zero-argument form simplifies the common pattern of
     *       pausing and resuming discovery without managing the service name externally.
     *
     * @warning **No Service Name:** If called without a prior StartDiscovery(const char*) call,
     *          the internal service name pointer may be nullptr, causing undefined behavior.
     *          Always ensure a valid service name is set first.
     *
     * @see StartDiscovery(const char* ServiceName) to set the service name initially
     * @see StopDiscovery() to pause discovery
     */
    virtual bool StartDiscovery() = 0;

    /**
     * @brief Stops service discovery and unregisters the service.
     * @details Disables mDNS service discovery for this channel.
     *          Unregisters the advertised service from the local network.
     *          After this call, remote devices will no longer discover this service.
     *
     *          **Network Impact:**
     *          - Service is removed from mDNS announcements
     *          - Existing client connections are NOT affected (only discovery stops)
     *          - Remote devices relying on mDNS will detect the service has gone offline
     *
     * @post Service is no longer advertised.
     *       - mDNS queries will not find this service
     *       - Existing TCP connections remain active (unaffected by discovery status)
     *       - StartDiscovery() can be called again to re-enable
     *
     * @note **Connection Independence:** Stopping discovery does not close active TCP connections.
     *       Remote clients with existing connections can continue communicating directly using
     *       the IP address they discovered earlier. Only new discovery operations are affected.
     *
     * @note **Idempotency:** Calling StopDiscovery() multiple times is safe; subsequent calls do nothing.
     *
     * @note **Graceful Shutdown:** StopDiscovery() is part of clean shutdown procedures.
     *       Typically called before Stop() to allow the service time to unregister gracefully.
     *
     * @note **Active Destruction:** In some implementations, the service unregistration takes
     *       a moment to propagate (< 1 second). For clean shutdown, stop discovery before
     *       disconnecting all clients.
     *
     * @note **Thread-Safety:** The implementation must handle potential concurrent calls.
     *       For FreeRTOS platforms, internal mutexes protect static mDNS state.
     *
     * @warning **Does Not Close Connections:** StopDiscovery() prevents NEW devices from discovering
     *          the service, but does NOT disconnect existing clients. To fully disconnect, call Stop()
     *          after StopDiscovery() if needed.
     *
     * @see StartDiscovery(const char* ServiceName) to re-enable discovery
     * @see StartDiscovery() to re-enable with the stored service name
     */
    virtual void StopDiscovery() = 0;

protected:
    /**
     * @brief Protected constructor.
     * @details Initializes a GenericTCP instance with a port number for TCP communication.
     *          Protected because this is an abstract base class; direct instantiation is not intended.
     *          Derived concrete classes (ESP32TCP, etc.) call this constructor.
     *
     * @param port The TCP port number for communication.
     *             Each TCP connection is uniquely identified by (IP address, port) pair.
     *             - Valid range: 1-65535
     *             - Well-known ports (0-1023): Reserved for system services; avoid these
     *             - Registered ports (1024-49151): Available for applications (preferred range)
     *             - Dynamic/private ports (49152-65535): Temporary ports (usually assigned by OS)
     *
     *             Common choices for robotics:
     *             - 5000: Common for custom applications
     *             - 8080: Alternative to 80 (HTTP)
     *             - 9000-9999: Good range for robotics/research applications
     *
     * @post `_port` is initialized to the provided port number.
     * @post `BaseChannel::_started` is initialized to false (by BaseChannel constructor).
     *
     * @note **Port Selection:**
     *       - Choose a port in the registered range (1024-49151) to avoid conflicts
     *       - Document your port choice in application configuration/README
     *       - Avoid common ports (80, 443, 8080) to prevent accidental conflicts
     *
     * @warning **Port Already in Use:** If the specified port is already in use by another service,
     *          the Start() method will fail. Use `netstat` or similar tools to check port availability.
     *
     * @warning **Firewall Blocking:** Routers and firewalls may block custom ports.
     *          Ensure network policies allow traffic on your chosen port.
     *          Port forwarding may be required for external access.
     *
     * @warning **Port 0 (Dynamic Assignment):** Some implementations may allow port 0, which
     *          instructs the OS to assign an available port automatically. Check derived class
     *          documentation for this behavior.
     */
    GenericTCP(uint16_t port) : _port(port), _serviceName(nullptr), BaseChannel() {};

    /**
     * @brief The TCP port number for communication.
     * @details Stored for reference and configuration of the underlying socket.
     *          Derived classes read this value and bind/connect to this port.
     * @note This is a read-only member once set in the constructor.
     * @note Units: Port number (1-65535).
     * @note A complete TCP endpoint address requires both this port and an IP address.
     */
    uint16_t _port;

    /**
     * @brief Pointer to the service name for mDNS discovery.
     * @details Stores a pointer to the human-readable name used for network service discovery.
     *          This name is registered with the mDNS/Bonjour service to allow remote devices
     *          to automatically discover this TCP channel without needing to know the IP address.
     *
     *          **Lifetime Management:**
     *          The pointer is stored directly, not copied. The caller is responsible for ensuring
     *          the string remains valid for the entire lifetime of the discovery registration.
     *          Typically, this points to a string literal or a statically-allocated buffer.
     *
     *          **Initial State:**
     *          - Initialized to nullptr in the constructor
     *          - Set by StartDiscovery(const char* ServiceName)
     *          - Remains valid until StopDiscovery() is called
     *          - Can be reused across multiple StartDiscovery() / StopDiscovery() cycles
     *
     * @note Valid service name characters (DNS label): alphanumeric, hyphens, underscores
     * @note Maximum length: 63 characters (DNS label size limit)
     * @note Examples: "robot-1", "arm-controller", "drone-01"
     * @note Thread-safe access is handled by derived class mutex protection
     *
     * @see StartDiscovery(const char* ServiceName) sets this pointer
     * @see StartDiscovery() uses this stored pointer
     * @see StopDiscovery() invalidates discovery but may keep pointer for later reuse
     */
    const char* _serviceName;
};

/**
 * @brief Smart handle (smart pointer) for GenericTCP instances.
 * @details Manages the lifetime of TCP channel objects, enabling automatic cleanup
 *          and safe sharing among multiple subsystems.
 *
 * @note Using smart pointers allows automatic resource cleanup when the last reference is released.
 */
using GenericTCPHandle = ChannelPointer(GenericTCP);

} // namespace Motion::Core::IO