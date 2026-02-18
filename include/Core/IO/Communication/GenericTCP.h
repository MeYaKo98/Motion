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
 *          for platform-specific implementations (e.g., ESP32TCP, LinuxTCP) that handle
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
    GenericTCP(uint16_t port) : _port(port), BaseChannel() {};

    /**
     * @brief The TCP port number for communication.
     * @details Stored for reference and configuration of the underlying socket.
     *          Derived classes read this value and bind/connect to this port.
     * @note This is a read-only member once set in the constructor.
     * @note Units: Port number (1-65535).
     * @note A complete TCP endpoint address requires both this port and an IP address.
     */
    uint16_t _port;
};

/**
 * @brief Smart handle (shared_ptr) for GenericTCP instances.
 * @details Manages the lifetime of TCP channel objects, enabling automatic cleanup
 *          and safe sharing among multiple subsystems.
 *
 * @note Using shared_ptr allows multiple parts of the application to reference the same
 *       TCP connection safely, with automatic resource cleanup when the last reference is released.
 */
using GenericTCPHandle = ChannelPointer(GenericTCP);

} // namespace Motion::Core::IO