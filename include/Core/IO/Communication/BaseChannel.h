/**
 * @file BaseChannel.h
 * @brief Defines the abstract base interface for communication channels.
 * @details This file contains the BaseChannel class, which serves as a foundation for specific communication
 *          channel implementations (e.g., Serial UART, Bluetooth, TCP/IP).
 *          It defines the contract for connection management and data transmission.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <memory>

namespace Motion::Core::IO {

/**
 * @brief Defining the type of pointers used for channel handles
 * @details Uses smart pointers for automatic lifetime management and safe multi-threaded access.
 */
#define ChannelPointer(T) std::shared_ptr<T>

/**
 * @class BaseChannel
 * @brief Abstract base class for all communication channels.
 * @details This interface standardizes how the application interacts with different communication
 *          mediums. Concrete implementations must implement the pure virtual methods to handle
 *          hardware-specific details like UART configuration, TCP socket management, or Bleutooth.
 *
 *          **Typical Communication Flow:**
 *          1. Create a concrete channel instance (ESP32Serial, ESP32TCP, etc.)
 *          2. Call `Start()` to initialize the hardware
 *          3. Send data with `Send()` and receive with `Read()`
 *          4. Check connection status with `IsConnected()`
 *          5. Call `Stop()` to clean up
 *
 *          **Example:**
 *          ```cpp
 *          auto serial = ESP32Serial::Create(Serial, 115200);
 *          if (serial->Start()) {
 *              uint8_t message[] = "Hello";
 *              serial->Send(message, 5);
 *              uint8_t rxBuffer[256];
 *              size_t bytesRead = serial->Read(rxBuffer, 256);
 *          }
 *          serial->Stop();
 *          ```
 *
 * @note **Concurrency:** Implementations may or may not be thread-safe. Document thread-safety
 *       guarantees at the derived class level.
 *
 * @note **Blocking Behavior:** Send/Read operations may be blocking or non-blocking depending on
 *       the underlying hardware and configuration. Document blocking behavior in derived classes.
 *
 * @see ESP32Serial for UART serial communication
 * @see ESP32TCP for TCP/IP socket communication
 */
class BaseChannel {
public:
    /**
     * @brief Starts the communication channel.
     * @details Performs necessary hardware initialization, resource allocation, or connection establishment.
     *          After calling this method, the channel should be ready to send and receive data
     *          (unless an error occurs).
     *
     *          Implementation details vary by channel type:
     *          - **Serial:** Configures baud rate, parity, stop bits
     *          - **TCP:** Opens socket wait for client to connect
     *          - **Bluetooth:** Pairs with remote device, establishes link
     *
     * @return true if the channel started successfully and is ready for use.
     *         false if initialization failed (e.g., port already in use, no network connectivity).
     *
     * @post If this method returns true, `IsConnected()` should return true, and `Send()`/`Read()`
     *       should be operable (subject to the hardware's readiness).
     *
     * @post Internal state `_started` is set to true on successful start.
     *
     * @note **Idempotency:** Calling `Start()` twice without an intervening `Stop()` may have
     *       different behavior depending on the implementation. Some implementations may silently
     *       succeed, others may fail or reinitialize resources.
     *
     * @note **Blocking:** This method may block briefly while initializing hardware or waiting
     *       for connection establishment. For time-sensitive applications, consider calling from
     *       a low-priority task.
     *
     * @note **Resource Allocation:** This method allocates resources (e.g., file descriptors, memory buffers).
     *       Always call `Stop()` to release these resources and prevent leaks.
     *
     * @warning **Hardware Availability:** Start may fail if the hardware is:
     *          - In use by another part of the application
     *          - Not configured with correct pins or addresses
     *          - Not physically connected or initialized
     *          Document specific error conditions in derived implementations.
     *
     * @warning **External Dependencies:** Before calling `Start()`, ensure external hardware is ready:
     *          - Serial port drivers are loaded
     *          - Network is active (for TCP)
     *          - Paired devices are in range (for Bluetooth)
     *
     * @see Stop()
     */
    virtual bool Start() = 0;

    /**
     * @brief Sends data over the communication channel.
     * @details Transmits the specified buffer over the channel. The method is responsible for
     *          properly formatting and error-checking the data as required by the specific channel type.
     *
     *          The caller is responsible for ensuring the buffer is valid and contains the intended data.
     *          The channel does not interpret or validate the data content—it is treated as raw bytes.
     *
     * @param[in] data Pointer to the buffer containing the data to send. Must not be nullptr.
     *                 The buffer must contain valid data for at least `length` bytes.
     * @param[in] length The number of bytes to send from the buffer.
     *                   Must be > 0 and consistent with the actual buffer size.
     *                   Passing length > actual buffer size results in a buffer overrun (undefined behavior).
     *
     * @return size_t The actual number of bytes sent successfully.
     *                - **Success:** Returns `length` if all bytes were sent.
     *                - **Partial Send:** Returns < `length` if only some bytes were sent
     *                   (e.g., due to buffer fullness). Caller may retry the remaining bytes.
     *                - **Failure:** Returns 0 if no bytes were sent (e.g., channel disconnected).
     *
     * @pre The channel must be started via `Start()` before calling this method.
     *
     * @pre The `data` pointer must point to valid, allocated memory.
     *
     * @note **Blocking Behavior:** The method may block if the underlying hardware buffer is full.
     *       Document blocking behavior in derived implementations, and consider timeout mechanisms.
     *
     * @note **Buffering:** The channel may buffer data internally before transmission.
     *       Use `Start()` or a flush method to ensure data is actually transmitted, if applicable.
     *
     * @note **Data Integrity:** Some channels (e.g., TCP) provide error checking and retransmission.
     *       Others (e.g., UART) rely on higher-layer protocols.
     *
     * @note **Partial Sends:** For large messages, the method may return fewer bytes than requested.
     *       The caller must loop and retry for any unsentbytes or use a higher-level protocol.
     *
     * @note **Non-Blocking Mode:** Some implementations may provide non-blocking variants.
     *       Document which mode is implemented or provide both.
     *
     * @warning **Null Pointer:** Passing `data == nullptr` results in undefined behavior (likely a crash).
     *          Always validate pointers before calling.
     *
     * @warning **Buffer Overrun:** Passing `length` greater than the actual buffer size causes
     *          a buffer overrun and **undefined behavior**. Always verify buffer size before calling.
     *
     * @warning **Channel Disconnection:** If the channel becomes disconnected during transmission
     *          (e.g., USB unplugged, TCP connection closed), subsequent calls return 0,
     *          indicating failure. Check `IsConnected()` after failures.
     *
     * @warning **Encoding/Framing:** The channel does not interpret data encoding.
     *          Upper layers are responsible for protocol framing (start/stop bits, delimiters, checksums).
     *
     * @see Read() for receiving data
     * @see IsConnected() to check if the channel is active
     */
    virtual size_t Send(const uint8_t* data, size_t length) = 0;

    /**
     * @brief Reads data from the communication channel.
     * @details Attempts to retrieve received data from the channel's input buffer.
     *          The method returns immediately (non-blocking) with the number of bytes available.
     *          If no data is available, it returns 0 without blocking.
     *
     *          The channel internally buffers received data. This method drains the buffer
     *          up to the capacity of the provided output buffer.
     *
     * @param[out] buffer Pointer to the destination buffer where read data will be stored.
     *                    Must not be nullptr. Must be large enough to hold `bufferSize` bytes.
     * @param[in] bufferSize The maximum capacity of the destination buffer in bytes.
     *                       The method will not read more than this many bytes.
     *                       Must be > 0 and match the actual buffer allocation.
     *
     * @return size_t The number of bytes actually read from the channel.
     *                - **Data Available:** Returns the number of bytes read, which may be any value
     *                   from 1 to `bufferSize` depending on buffering and timing.
     *                - **No Data:** Returns 0 if no data is currently available.
     *                   This does not indicate an error, just that there is no new data yet.
     *                - **Error:** Some implementations may return 0 on error; others may return a special
     *                   error code. Consult the derived implementation documentation.
     *
     * @pre The channel must be started via `Start()` before calling this method.
     *
     * @pre The `buffer` pointer must point to valid, allocated memory of at least `bufferSize` bytes.
     *
     * @post The returned bytes are removed from the internal buffer (consumed).
     *       Calling `Read()` again will retrieve any remaining bytes.
     *
     * @note **Non-Blocking:** This method is typically non-blocking and returns immediately.
     *       It does not wait for data. If you need to wait for data, implement polling or
     *       callbacks at a higher level.
     *
     * @note **Buffering:** The channel internally buffers received data in a queue or circular buffer.
     *       Very high data rates may overflow the buffer if `Read()` is not called frequently enough.
     *
     * @note **Partial Reads:** The method may return fewer bytes than the buffer capacity.
     *       This is normal and does not indicate an error. Call `Read()` again to retrieve more data.
     *
     * @note **Data Ordering:** Multiple calls to `Read()` will return bytes in the order they were
     *       received (FIFO). No reordering or buffering at the application level is needed.
     *
     * @note **Hardware Specific:** Some channels (e.g., Serial, TCP) may have platform-specific
     *       buffer sizes. Very high baud rates or message frequencies may require larger buffers.
     *
     * @warning **Null Pointer:** Passing `buffer == nullptr` results in undefined behavior (likely crash).
     *          Always validate pointers.
     *
     * @warning **Buffer Overrun:** Passing `bufferSize` without ensuring the buffer is actually
     *          that large results in **undefined behavior**. Always allocate the correct size.
     *
     * @warning **Data Loss:** If the internal buffer overflows (reads called too infrequently),
     *          data is lost silently. Implement monitoring for this condition if critical.
     *
     * @warning **Zero Buffer Size:** Passing `bufferSize == 0` results in undefined behavior.
     *          Always pass bufferSize > 0.
     *
     * @see Send() for transmitting data
     * @see IsConnected() to check channel status
     */
    virtual size_t Read(uint8_t* buffer, size_t bufferSize) = 0;

    /**
     * @brief Checks if the channel is currently connected and operational.
     * @details Verifies that the underlying communication medium is active and ready for sending/receiving.
     *          The interpretation of "connected" depends on the channel type:
     *          - **Serial:** Device file is open and port is initialized
     *          - **TCP:** Socket is open and client is connected
     *          - **Bluetooth:** Link is paired and active
     *
     * @return true if the channel is connected and valid.
     *         false if the channel is disconnected, not started, or has encountered an error.
     *
     * @note **Snapshot:** This method returns a **snapshot** of the connection state at the time
     *       of the call. The state may change immediately after this method returns.
     *       In multi-threaded environments, always check immediately before critical operations.
     *
     * @note **Error Recovery:** If `IsConnected()` returns false, call `Stop()` and `Start()` again
     *       to attempt recovery.
     *
     * @note **Frequency:** It is safe to call this method frequently (every iteration of a loop)
     *       without significant overhead.
     *
     * @warning **Race Condition:** In multi-threaded environments, the state may change between
     *          the call to `IsConnected()` and the next `Send()`/`Read()` call.
     *          Design error handling to be resilient to this.
     *
     * @see Start()
     * @see Stop()
     */
    virtual bool IsConnected() = 0;

    /**
     * @brief Stops the communication channel.
     * @details Closes connections and releases hardware resources. After calling this method,
     *          the channel is no longer usable until `Start()` is called again.
     *
     *          Implementation details vary by channel type:
     *          - **Serial:** Closes file descriptors, releases UART hardware
     *          - **TCP:** Closes sockets, terminates network connection
     *          - **Bluetooth:** Closes link
     *
     * @post The channel is no longer connected. `IsConnected()` will return false.
     *       `Send()` and `Read()` calls will fail or return 0.
     *
     * @post Internal state `_started` is set to false.
     *
     * @post All hardware resources allocated by `Start()` are released.
     *
     * @note **Resource Cleanup:** Always call this method when finished with a channel to prevent
     *       resource leaks (file descriptor exhaustion, memory leaks, hardware deadlock).
     *
     * @note **Idempotency:** Calling `Stop()` multiple times is safe—subsequent calls do nothing.
     *
     * @note **No Error Handling:** This method typically does not report errors.
     *       Assume it always succeeds in releasing resources.
     *
     * @note **RAII Pattern:** Consider wrapping `BaseChannel` in a RAII wrapper to automatically
     *       call `Stop()` in the destructor.
     *
     * @warning **Use-After-Stop:** Calling `Send()` or `Read()` after `Stop()` results in
     *          failure or undefined behavior. Always check `IsConnected()` before these operations.
     *
     * @see Start()
     */
    virtual void Stop() = 0;

    /**
     * @brief Virtual destructor.
     * @details Allows proper cleanup of derived class instances when deleted through a base class pointer.
     *          The default implementation does not perform any special cleanup beyond virtual method dispatch.
     */
    virtual ~BaseChannel() = default;

    /**
     * @brief Deleted copy constructor to prevent copying of hardware resources.
     * @details Hardware communication channels represent system resources that should not be duplicated.
     *          Copying would create multiple references to the same hardware, leading to conflicts
     *          and concurrency issues.
     */
    BaseChannel(const BaseChannel&) = delete;

    /**
     * @brief Deleted assignment operator to prevent assignment of hardware resources.
     * @details Prevents unintended resource sharing and state corruption from assignment operations.
     */
    BaseChannel& operator=(const BaseChannel&) = delete;

protected:
    /**
     * @brief Default constructor.
     * @details Initializes the channel state to not started (`_started = false`).
     *          Derived classes should call this constructor.
     */
    BaseChannel() : _started(false) {};
    
    /**
     * @brief Internal flag tracking the started state of the channel.
     * @details Initially false. Set to true by `Start()` and false by `Stop()`.
     *          Some derived implementations may check this flag before allowing Send/Read.
     * @note This is a basic flag. Some implementations may have additional connection state tracking.
     */
    bool _started;
};

/**
 * @brief Smart handle for BaseChannel instances using smart pointers.
 * @details Manages the lifetime of channel objects, ensuring proper cleanup when no longer referenced.
 */
using BaseChannelHandle = ChannelPointer(BaseChannel);

} // namespace Motion::Core::IO