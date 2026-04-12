#include "Motion/Core/Logger.h"
#include <stdarg.h>
#include <stdio.h>
#include <ctype.h>

namespace Motion::Core {

bool Logger::Start(Motion::Core::IO::BaseChannelHandle& channelHandle, LogLevel minLevel, uint32_t queueSize)
{
    if (_msgQueue != nullptr)
    {
        LOG_WARN("Logger already started!");
        return true;
    }

    if (!channelHandle)
    {
        throw std::invalid_argument("Channel handle cannot be null");
        return false;
    }
    
    if (!channelHandle->Start())
    {
        throw std::runtime_error("Failed to start communication channel");
        return false;
    }

    _msgQueue = xQueueCreate(queueSize, sizeof(LogMessage));
    if (!_msgQueue)
    {
        throw std::runtime_error("Failed to create message queue");
        return false;
    }    

    _minLevel = minLevel;
    _outputHandle = channelHandle;
    
    if (!xTaskCreate(LogTask, "LoggerTask", 4096, this, 1, nullptr))
    {
        throw std::runtime_error("Failed to create the logger task");
        return false;
    }

    return true;
}

void Logger::Log(LogLevel level, const char* tag, const char* file, int line, const char* format, ...)
{
    if (!_msgQueue || level < _minLevel) return;
    if (!tag || !file || !format) return; 

    LogMessage msg;
    uint32_t timestamp = (xPortInIsrContext()) ? 0 : xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Header: [ms] [LEVEL] [Tag] (file:line): 
    int headerLen = snprintf(msg.data, sizeof(msg.data) - 1, 
                             "[%07lu] [%s] [%s] (%s:%d): ", 
                             timestamp, GetLevelLabel(level), tag, file, line);

    va_list args;
    va_start(args, format);
    int bodyLen = vsnprintf(msg.data + headerLen, sizeof(msg.data) - 1 - headerLen, format, args);
    va_end(args);

    if (bodyLen < 0) return;
    int totalLen = headerLen + bodyLen;

    if (totalLen > (int)sizeof(msg.data) - 1) totalLen = sizeof(msg.data) - 1;

    msg.data[totalLen] = '\n';
    msg.length = totalLen + 1;

    if (xPortInIsrContext())
    {
        BaseType_t woken = pdFALSE;
        xQueueSendFromISR(_msgQueue, &msg, &woken);
        if (woken) portYIELD_FROM_ISR();
    } else
    {
        xQueueSend(_msgQueue, &msg, 0);
    }
}

const char* Logger::GetLevelLabel(LogLevel level)
{
    switch (level)
    {
        case LogLevel::TRACE: return "TRACE";
        case LogLevel::DEBUG: return "DEBUG";
        case LogLevel::INFO:  return "INFO";
        case LogLevel::WARN:  return "WARN";
        case LogLevel::ERROR: return "ERROR";
        default: return "LOG";
    }
}

void Logger::LogTask(void* pvParameters)
{
    Logger* self = (Logger*)pvParameters;
    LogMessage receivedMsg;
    while (true)
    {
        if (xQueueReceive(self->_msgQueue, &receivedMsg, portMAX_DELAY) == pdPASS)
        {
            self->_outputHandle->Send((const uint8_t*)receivedMsg.data, receivedMsg.length);
        }
    }
}

} // namespace Motion::Core