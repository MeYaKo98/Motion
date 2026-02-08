/**
 * @file Logger.h
 * @brief Header file for the Logger class and associated macros.
 */

#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <string.h>
#include "Core/IO/Communication/BaseChannel.h"

namespace Motion::Core::Diagnostics {

/**
 * @enum LogLevel
 * @brief Defines the severity levels for log messages.
 */
enum class LogLevel : uint8_t {
    TRACE = 0, /**< Detailed tracing information for debugging flow. */
    DEBUG = 1, /**< Debugging information useful for developers. */
    INFO  = 2, /**< General informational messages about system state. */
    WARN  = 3, /**< Warnings about potential issues that are not immediately fatal. */
    ERROR = 4  /**< Critical errors indicating a failure in a component. */
};

/**
 * @class Logger
 * @brief Singleton logger class that handles message formatting, buffering, and asynchronous output.
 * @details The Logger uses a FreeRTOS queue to buffer messages from various tasks or ISRs
 *          and a dedicated background task to dispatch them to the configured output channel.
 *          This design ensures that logging calls are non-blocking (mostly) and ISR-safe.
 */
class Logger {
public:
    /**
     * @brief Retrieves the singleton instance of the Logger.
     * @return Logger& Reference to the global Logger instance.
     */
    static Logger& GetInstance() {
        static Logger instance;
        return instance;
    }

    /**
     * @brief Initializes the logging system and starts the background logging task.
     * 
     * @param channel Pointer to the communication channel used for output (e.g., Serial, TCP).
     *                The logger takes ownership of using this channel but not its memory management.
     *                Must not be nullptr.
     * @param minLevel The minimum severity level to log. Messages below this level are ignored.
     *                 Defaults to LogLevel::INFO.
     * @param queueSize The maximum number of messages the internal queue can hold.
     *                  Defaults to 30.
     * 
     * @warning This method must be called only once during system initialization.
     * @warning The provided `channel` must be initialized and valid. Passing nullptr will cause a crash.
     * 
     * @remark It is recommended to use the helper macros.
     * @see LOG_START
     */
    void Start(Motion::Core::IO::BaseChannel* channel, LogLevel minLevel = LogLevel::INFO, uint32_t queueSize = 30);

    /**
     * @brief Queues a log message for processing.
     * @details Formats the message with timestamp, level, tag, and file location, then pushes it
     *          to the queue. If called from an ISR, it uses the ISR-safe queue functions.
     * 
     * @param level The severity level of the message.
     * @param tag A short string tag identifying the source (usually the function name).
     * @param file The source file name where the log occurred.
     * @param line The line number in the source file.
     * @param format A printf-style format string.
     * @param ... Variable arguments for the format string.
     * 
     * @note This function is safe to call from Interrupt Service Routines (ISRs).
     * @note If the queue is full, the message will be dropped to prevent blocking the calling task/ISR.
     * 
     * @remark It is recommended to use the helper macros for logging to automatically capture metadata.
     * @see LOG_TRACE
     * @see LOG_DEBUG
     * @see LOG_INFO
     * @see LOG_WARN
     * @see LOG_ERROR
     */
    void Log(LogLevel level, const char* tag, const char* file, int line, const char* format, ...);

    Logger(Logger const&) = delete;
    void operator=(Logger const&) = delete;

private:
    /**
     * @brief Private constructor to enforce Singleton pattern.
     */
    Logger() : _output(nullptr), _msgQueue(nullptr), _minLevel(LogLevel::INFO) {}

    /**
     * @brief The FreeRTOS task function that processes queued log messages.
     * @param pvParameters Pointer to the Logger instance (this).
     */
    static void LogTask(void* pvParameters);

    /**
     * @struct LogMessage
     * @brief Internal structure representing a queued log message.
     * @details Holds the formatted log string and its length. Used to pass data
     *          from the logging call site to the background worker task.
     */
    struct LogMessage {
        char data[256]; /**< Buffer for the formatted log message including metadata. */
        uint16_t length; /**< Length of the valid data in the buffer. */
    };

    /**
     * @brief Helper to convert LogLevel enum to a 3-character string label.
     * @param level The log level to convert.
     * @return const char* String representation (e.g., "INFO", "ERROR").
     */
    const char* GetLevelLabel(LogLevel level);

    Motion::Core::IO::BaseChannel* _output;
    QueueHandle_t _msgQueue;
    LogLevel _minLevel;
};

} // namespace Motion::Core::Diagnostics

/**
 * @brief Macro to start the logger instance.
 * @see Logger::Start
 */
#define LOG_START(...) Motion::Core::Diagnostics::Logger::GetInstance().Start(__VA_ARGS__)

/**
 * @brief Internal base macro for logging.
 * @details Automatically captures the function name, file name, and line number.
 */
#define _LOG_BASE(lvl, fmt, ...) Motion::Core::Diagnostics::Logger::GetInstance().Log(lvl, __FUNCTION__, __FILENAME__, __LINE__, fmt, ##__VA_ARGS__)

/** @brief Logs a TRACE level message. */
#define LOG_TRACE(fmt, ...) _LOG_BASE(Motion::Core::Diagnostics::LogLevel::TRACE, fmt, ##__VA_ARGS__)
/** @brief Logs a DEBUG level message. */
#define LOG_DEBUG(fmt, ...) _LOG_BASE(Motion::Core::Diagnostics::LogLevel::DEBUG, fmt, ##__VA_ARGS__)
/** @brief Logs an INFO level message. */
#define LOG_INFO(fmt, ...)  _LOG_BASE(Motion::Core::Diagnostics::LogLevel::INFO,  fmt, ##__VA_ARGS__)
/** @brief Logs a WARN level message. */
#define LOG_WARN(fmt, ...)  _LOG_BASE(Motion::Core::Diagnostics::LogLevel::WARN,  fmt, ##__VA_ARGS__)
/** @brief Logs an ERROR level message. */
#define LOG_ERROR(fmt, ...) _LOG_BASE(Motion::Core::Diagnostics::LogLevel::ERROR, fmt, ##__VA_ARGS__)