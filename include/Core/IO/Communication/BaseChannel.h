/**
 * @file BaseChannel.h
 * @brief Defines the abstract base interface for communication channels.
 * @details This file contains the BaseChannel class. It serves as a foundation for specific communication
 *          channel implementations (e.g., Serial, Bluetooth, TCP/IP).
 *          It defines the contract for connection management and data transmission.
 */


#pragma once

#include <stdint.h>
#include <stddef.h>
#include <memory>

namespace Motion::Core::IO {

/**
 * @brief Defining The type of pointers used fr channel handles (shared_ptr)
 */
#define ChannelPointer(T) std::shared_ptr<T>

/**
 * @class BaseChannel
 * @brief Abstract base class for all communication channels.
 * @details This interface standardizes how the application interacts with different communication
 *          mediums. Concrete implementations must implement the pure virtual methods to handle
 *          hardware-specific details.
 */
class BaseChannel {
public:
    /**
     * @brief Starts the communication channel.
     * @details Performs necessary hardware initialization, resource allocation, or connection establishment.
     * @return true if the channel started successfully and is ready for use.
     * @return false if initialization failed.
     */
    virtual bool Start() = 0;

    /**
     * @brief Sends data over the communication channel.
     * @param[in] data Pointer to the buffer containing the data to send. Must not be nullptr.
     * @param[in] length The number of bytes to send from the buffer.
     * @return size_t The actual number of bytes sent.
     */
    virtual size_t Send(const uint8_t* data, size_t length) = 0;

    /**
     * @brief Reads data from the communication channel.
     * @param[out] buffer Pointer to the destination buffer where read data will be stored. Must not be nullptr.
     * @param[in] bufferSize The maximum capacity of the destination buffer in bytes.
     * @return size_t The number of bytes actually read. Returns 0 if no data is available.
     */
    virtual size_t Read(uint8_t* buffer, size_t bufferSize) = 0;

    /**
     * @brief Checks if the channel is currently connected.
     * @return true if the channel is connected and valid.
     * @return false if the channel is disconnected.
     */
    virtual bool IsConnected() = 0;

    /**
     * @brief Stops the communication channel.
     * @details Closes connections and releases hardware resources.
     */
    virtual void Stop() = 0;

    /**
     * @brief Virtual destructor.
     */
    virtual ~BaseChannel() = default;

    /**
     * @brief Deleted copy constructor to prevent copying of hardware resources.
     */
    BaseChannel(const BaseChannel&) = delete;

    /**
     * @brief Deleted assignment operator to prevent assignment of hardware resources.
     */
    BaseChannel& operator=(const BaseChannel&) = delete;

protected:
    /**
     * @brief Default constructor.
     * @details Initializes the channel state to not started.
     */
    BaseChannel() : _started(false) {};
    
    /**
     * @brief Internal flag tracking the started state of the channel.
     */
    bool _started;
};

/**
 * @brief Defining The base channel handle (unique_ptr)
 */
using BaseChannelHandle = ChannelPointer(BaseChannel);

} // namespace Motion::Core::IO