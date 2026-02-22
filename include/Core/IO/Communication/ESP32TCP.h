/**
 * @file ESP32TCP.h
 * @brief ESP32 WiFi TCP socket communication channel implementation.
 * @details Provides TCP/IP networking over WiFi for remote robot control and telemetry.
 */

#pragma once

#include "Core/IO/Communication/GenericTCP.h"
#include <WiFi.h>

namespace Motion::Core::IO {

class ESP32TCP;
    
/**
 * @brief Smart handle for ESP32TCP instances.
 * @details Uses smart pointers for automatic lifetime management and safe reference counting.
 */
using ESP32TCPHandle = ChannelPointer(ESP32TCP);

/**
 * @brief ESP32 WiFi TCP communication channel using Arduino WiFi API.
 * @details This class implements TCP/IP networking on the ESP32 using the Arduino WiFi library.
 *          It provides a socket-based communication interface for transmitting and receiving data
 *          over WiFi networks. Common uses include remote teleoperation, wireless logging, 
 *          inter-robot communication, and connection to cloud services.
 *
 *          **ESP32 WiFi Hardware:**
 *          - Integrated WiFi (802.11 b/g/n) and Bluetooth dual-mode radio
 *          - Supports both Station (client) and Access Point (AP) modes
 *          - Can operate in Station + AP simultaneous mode (rarely needed)
 *          - Data rates up to 150 Mbps (typically 50-100 Mbps in practice)
 *
 *          **Arduino WiFi API:**
 *          - `WiFiServer`: Listens for incoming TCP connections (server side)
 *          - `WiFiClient`: Represents a single connected client (or outgoing connection)
 *          - Blocking and non-blocking modes available
 *          - Automatic connection management (reconnects on failure)
 *
 *          **TCP Server Model (This Implementation):**
 *          This class implements a TCP server:
 *          1. WiFiServer listens on the configured port
 *          2. Client connects from remote host (e.g., PC, remote control station)
 *          3. WiFiServer accepts the connection, creating a WiFiClient object
 *          4. Send/Read operate on the WiFiClient connection
 *          5. Disconnecting resets the client; next incoming connection accepted
 *
 * @note **Blocking Behavior:** WiFiServer::available() and WiFiClient::read() can block.
 *       For real-time applications, consider implementing non-blocking wrappers or
 *       running network I/O in a dedicated FreeRTOS task with appropriate priorities.
 *
 * @note **Latency:** WiFi introduces variable latency (10-100 ms typical, up to 1000+ ms
 *       on congested networks). Not suitable for time-critical control (< 10 ms loops).
 *       Use wired connections (Ethernet via SPI) for deterministic timing.
 *
 * @note **Network Configuration:** Requires WiFi SSID and password setup. Typically done
 *       in setup() before creating ESP32TCP instance:
 * @code 
 * WiFi.begin("MY_SSID", "password");  // Connect to WiFi AP
 * while (WiFi.status() != WL_CONNECTED) delay(100);
 * // Now safe to create ESP32TCP server
 * @endcode
 *
 * @warning **WiFi Latency:** WiFi is inherently non-deterministic. Expect variable latency
 *          and occasional packet loss, even on good networks. Buffer or filter data accordingly.
 *
 * @warning **Security:** This implementation provides no encryption or authentication.
 *          Traffic is transmitted in plaintext over the network. For production, implement
 *          TLS/SSL encryption or run on a secure, isolated network.
 *
 * @see GenericTCP for the base TCP interface
 * @see BaseChannel for the general communication channel interface
 */
class ESP32TCP : public GenericTCP {
public:
    /**
     * @brief Factory method to create an ESP32TCP server channel.
     * @details Creates a new TCP server that listens for incoming connections on the specified port.
     *          The server accepts one client at a time. Multiple simultaneous clients are not
     *          supported by this simple implementation.
     *
     * @param port The TCP port number to listen on.
     *             Valid range: 1-65535
     *             Common choices for robotics: 5000, 8000, 9000-9999
     *             Avoid well-known ports (80, 443) and system-reserved ports (< 1024)
     *
     * @return ESP32TCPHandle A smart pointer to the newly created ESP32TCP instance.
     *
     * @post The instance is created but **NOT started**. Call Start() to bind to the port
     *       and listen for incoming connections.
     *
     * @note **Usage Pattern:**
     * @code
     * // Ensure WiFi is connected first
     * WiFi.begin("SSID", "password");
     * while (WiFi.status() != WL_CONNECTED) delay(100);
     * auto tcp = ESP32TCP::Create(5000);
     * if (tcp->Start()) {
     *       // Now listening for connections
     *       uint8_t cmd[10];
     *       size_t len = tcp->Read(cmd, 10);   // Wait for client command
     *       tcp->Send(cmd, len);               // Echo back
     * }
     * tcp->Stop();
     * @endcode
     *
     * @warning WiFi must be initialized and connected before creating ESP32TCP.
     *          Ensure `WiFi.begin()` has been called and `WiFi.status() == WL_CONNECTED`
     *          before calling Create() and Start().
     *
     * @warning No validation of port number is performed. Invalid ports (0, > 65535) may
     *          cause Start() to fail silently.
     */
    static ESP32TCPHandle Create(uint16_t port);

    /**
     * @brief Destroys the ESP32TCP server channel.
     * @details Calls Stop() to close the server and any active client connections.
     *          Releases all WiFi resources.
     *
     * @note The destructor automatically calls Stop(), so explicit Stop() is optional.
     */
    virtual ~ESP32TCP();

    /**
     * @brief Starts the TCP server and listens for incoming connections.
     * @details Creates a WiFiServer on the specified port and configures it to listen
     *          for incoming TCP connections. Returns true once listening is active.
     *          Does NOT block waiting for a client; Start() returns immediately.
     *
     *          **Initialization:**
     *          1. WiFiServer initialized with configured port
     *          2. `server.begin()` called to start listening
     *          3. `_started` flag set to true
     *          4. Ready to accept incoming client connections
     *
     * @return true if the server successfully started and is listening for connections.
     * @return false if startup failed (e.g., port in use, network not ready).
     *
     * @post On success:
     *       - Server is listening on the configured port
     *       - `IsConnected()` returns true (server is active)
     *       - Clients can connect from remote hosts
     *       - Acceptance of first client happens automatically in the next Read/Send call
     *
     * @post On failure:
     *       - Server is not listening
     *       - `_started` = false
     *       - `IsConnected()` returns false
     *
     * @note **Non-Blocking:** Start() returns immediately; does not wait for a client connection.
     *       The server is ready to receive connections, but no client is yet connected.
     *
     * @note **Automatic Client Acceptance:** When Read() or Send() is called after Start(),
     *       the server automatically accepts the next available client connection. This happens
     *       transparently; the caller doesn't need to explicitly accept connections.
     *
     * @note **Single Client Model:** Only one client connection is maintained at a time.
     *       If a second client connects while one is active, the new connection is rejected.
     *
     * @note **Idempotency:** Calling Start() multiple times is typically safe; the second
     *       call reconfigures the server and returns success.
     *
     * @warning **Network Requirement:** ESP32 must be connected to a WiFi network before calling Start().
     *          Ensure `WiFi.status() == WL_CONNECTED` before this call. Starting without a network
     *          connection may fail silently.
     *
     * @warning **Port Conflicts:** If another process is already listening on the port, Start() fails.
     *          Use `netstat` on your network to check port availability before deploying.
     *
     * @warning **Firewall Blocking:** Some routers/firewalls block incoming TCP connections to
     *          non-standard ports. Ensure your network allows inbound TCP on the configured port.
     *
     * @see Stop()
     * @see IsConnected()
     */
    bool Start() override;

    /**
     * @brief Sends data to the connected TCP client.
     * @details Transmits the specified buffer over the active TCP connection.
     *          If no client is currently connected, the method waits briefly for a connection
     *          or returns without sending. Once a client connects, data reaches it reliably
     *          (TCP guarantees in-order, error-checked delivery).
     *
     *          **Data Flow:**
     *          1. WiFiClient.write() queues the data to the TCP transmit buffer
     *          2. TCP stack sends the data to the remote host
     *          3. Remote TCP stack acknowledges receipt
     *          4. Data is available for the remote application to read
     *
     * @param data Pointer to the buffer containing data to send.
     *             Must not be nullptr. Must point to valid, readable memory.
     *
     * @param length The number of bytes to send.
     *               Must be > 0 and consistent with the buffer size.
     *               Sending length > buffer_size causes buffer overrun (undefined behavior).
     *
     * @return size_t The number of bytes actually sent.
     *                - **Success:** Returns `length` if all bytes were queued for transmission
     *                - **Partial:** Returns < `length` if TCP buffer became full
     *                - **Failure:** Returns 0 if no client connected or send failed
     *
     *                Most applications should verify return value == length.
     *
     * @pre Start() must have been called successfully before this method.
     *      A client must be connected (or connection will be awaited).
     *
     * @post The data bytes are queued in the TCP transmit buffer.
     *       They will be sent to the remote client at network speed.
     *       TCP layer guarantees reliable, in-order delivery.
     *
     * @note **TCP Reliability:** Unlike UDP, TCP guarantees:
     *       - In-order delivery (bytes arrive in the order sent)
     *       - Error detection (corrupted packets are retransmitted)
     *       - Flow control (sender slows down if receiver is full)  
     *       This makes TCP ideal for command/control data.
     *
     * @note **Buffering:** The WiFiClient maintains a transmit buffer (typically 512 or 4096 bytes).
     *       If the remote client isn't reading quickly, the buffer fills and Send() blocks or
     *       returns partial length. Retry logic is recommended for large messages.
     *
     * @note **Non-Blocking Available:** Some implementations provide non-blocking modes via
     *       `WiFiClient::setNoDelay()` or similar. Check Arduino WiFi documentation.
     *
     * @note **Client Acceptance:** If Send() is called before any client has connected,
     *       the method briefly waits for an incoming connection (blocking). If no client connects
     *       within the timeout, Send() returns 0.
     *
     * @note **Thread-safety:** NOT thread-safe without external synchronization.
     *       Concurrent calls from multiple FreeRTOS tasks cause race conditions.
     *       Use a mutex to protect access if needed.
     *
     * @warning **Null Pointer:** Passing `data == nullptr` causes undefined behavior (crash).
     *
     * @warning **Buffer Overrun:** Passing `length > actual_buffer_size` causes buffer overrun
     *          and undefined behavior (memory corruption, crash).
     *
     * @warning **No Client Connected:** If Send() is called before Start() or with no client
     *          connected, the method times out and returns 0. Ensure Start() is called and a client
     *          has connected before relying on Send().
     *
     * @warning **Blocking Behavior:** WiFiClient.write() may block briefly if the TCP buffer is full.
     *          For a truly non-blocking implementation, use a separate task for network I/O.
     *
     * @see Read() for receiving data
     * @see IsConnected() to check if a client is connected
     */
    size_t Send(const uint8_t* data, size_t length) override;

    /**
     * @brief Reads data from the connected TCP client.
     * @details Attempts to retrieve received data from the TCP connection.
     *          If no client is currently connected, the method waits briefly for a connection.
     *          Once connected, data is read from the TCP receive buffer.
     *
     *          **Data Flow:**
     *          1. Remote client transmits data
     *          2. TCP stack receives and buffers the data
     *          3. Read() retrieves bytes from the TCP receive buffer
     *          4. Application processes the data
     *
     * @param buffer Pointer to the destination buffer for received data.
     *               Must not be nullptr. Must be large enough for `bufferSize` bytes.
     *               Allocated and managed by the caller.
     *
     * @param bufferSize The maximum number of bytes to read.
     *                   Actual bytes returned: min(available_bytes, bufferSize).
     *                   Must match the actual buffer allocation.
     *
     * @return size_t The number of bytes actually read.
     *                - **Data Available:** Returns 1 to `bufferSize` bytes
     *                - **No Data:** Returns 0 if no data is available from the client
     *                - **Client Disconnected:** Returns 0 (indistinguishable from no data)
     *
     * @pre Start() must have been called to enable the server.
     *      A client must be connected or will be awaited.
     *
     * @post The returned bytes have been removed from the receive buffer (consumed).
     *       Subsequent Read() calls retrieve any remaining bytes, or 0 if the buffer is empty.
     *
     * @note **Client Connection Acceptance:** If Read() is called before any client has connected,
     *       the method waits briefly for an incoming connection (blocking).
     *       If a connection arrives, data from that client is returned.
     *       If no connection arrives within the timeout, Read() returns 0.
     *
     * @note **Variable-Length Reads:** The number of bytes returned depends on timing.
     *       You may receive 1 byte, then 100, then 0. Higher-layer protocols must handle
     *       variable-length reads and frame reassembly.
     *
     * @note **TCP Buffering:** The TCP receive buffer (typically 512-4096 bytes) holds incoming data.
     *       If the client sends large amounts of data faster than Read() is called, the buffer may overflow.
     *       Call Read() frequently (> 100+ Hz) to avoid data loss.
     *
     * @note **Non-Blocking:** Read() returns immediately with available data or 0.
     *       It does not block waiting for new data. Implement polling loops or event callbacks
     *       if you need to wait for incoming data.
     *
     * @note **Client Disconnection:** If the remote client disconnects (connection closed),
     *       subsequent Read() calls return 0. No error is reported; reconnection handling
     *       must happen at the application layer.
     *
     * @note **Thread-safety:** NOT thread-safe without external synchronization.
     *       Concurrent calls to Read() from multiple tasks cause race conditions.
     *       Use a mutex to protect access if needed.
     *
     * @warning **Null Pointer:** Passing `buffer == nullptr` causes undefined behavior (crash).
     *
     * @warning **Buffer Overrun:** Passing `bufferSize` without allocating that memory
     *          causes buffer overrun and undefined behavior (memory corruption, crash).
     *
     * @warning **Zero Buffer Size:** Passing `bufferSize == 0` causes undefined behavior.
     *          Always allocate at least 1 byte if calling Read().
     *
     * @warning **No Blocking for Data:** Read() returns immediately if no data is available.
     *          To wait for data, implement polling or a FreeRTOS task event-driven model.
     *
     * @warning **Data Loss on Overflow:** If data arrives faster than Read() consumes it,
     *          the TCP buffer overflows and data is dropped. No error is reported.
     *          Monitor throughput and call Read() frequently to prevent this.
     *
     * @see Send() for transmitting data
     * @see IsConnected() to check connection status
     */
    size_t Read(uint8_t* buffer, size_t bufferSize) override;

    /**
     * @brief Checks if a TCP client is currently connected.
     * @details Checks if there is an active WiFiClient connection.
     *          Returns true if a client is connected and communicating.
     *
     * @return true if a client is currently connected and the connection is active.
     * @return false if no client is connected or if the server is not running.
     *
     * @note **State Snapshot:** This returns a snapshot of the current state.
     *       The connection may be lost immediately after this method returns.
     *       For critical operations, check immediately before Send/Read.
     *
     * @note **Non-Blocking:** This query is very fast (just checks a flag).
     *       Safe to call frequently.
     *
     * @warning **Not Network Health Check:** IsConnected() checks only that the server
     *          accepted a connection. It doesn't verify the underlying network is functional
     *          or that the remote host is still present. A true return means "we have a WiFiClient,"
     *          not "the device is reachable and responsive."
     *
     * @warning **Disconnection Delay:** If the remote client suddenly disconnects (e.g., crashes,
     *          network cable unplugged), IsConnected() may return true briefly before the system
     *          detects the failure. TCP keep-alive detection is slow (seconds to minutes).
     *
     * @see Start()
     * @see Stop()
     */
    bool IsConnected() override;

    /**
     * @brief Stops the TCP server and closes any active client connections.
     * @details Closes the WiFiServer, disconnects any active WiFiClient,
     *          and releases network resources. After Stop(), the server is no longer listening
     *          and no clients can connect until Start() is called again.
     *
     * @post The server is shut down:
     *       - WiFiServer stops listening
     *       - Any active client connection is closed
     *       - TCP resources are released
     *       - `IsConnected()` returns false
     *       - `Send()/Read()` return 0 (failure)
     *
     * @note **Idempotency:** Calling Stop() multiple times is safe; subsequent calls do nothing.
     *
     * @note **Resource Release:** Stop() ensures proper cleanup of WiFi TCP resources.
     *       Important for power management or restarting the server on a different port.
     *
     * @note **Called by Destructor:** The destructor automatically calls Stop().
     *       Explicit Stop() before object destruction is optional but recommended.
     *
     * @note **Data Loss:** Any data in send/receive buffers is discarded.
     *       Ensure all important data has been transmitted before Stop().
     *
     * @note **Restart:** After Stop(), call Start() to restart the server on the same port
     *       (or create a new instance for a different port).
     *
     * @warning **Timing:** If Stop() is called while data is being transmitted, the
     *          transmission is cut off. Allow brief time for pending TX if critical,
     *          or implement handshake protocols.
     *
     * @see Start()
     */
    void Stop() override;

protected:
    /**
     * @brief Constructs an ESP32TCP server instance.
     * @details Protected constructor; use the Create() factory method instead.
     *
     * @param port The TCP port number to listen on.
     *
     * @post The instance is initialized with the port number.
     *       Server is NOT yet listening; call Start() to begin.
     */
    ESP32TCP(uint16_t port);

private:
    /**
     * @brief WiFi server instance (listens for incoming TCP connections).
     * @details Created in Start(), destroyed in Stop().
     * @note Holds the listening socket and manages client acceptance.
     */
    WiFiServer _server;

    /**
     * @brief WiFi client instance (represents the active TCP connection).
     * @details Manages data transmission/reception for the currently connected client.
     * @note Becomes valid after a client connects to _server.
     */
    WiFiClient _client;
};

} // namespace Motion::Core::IO