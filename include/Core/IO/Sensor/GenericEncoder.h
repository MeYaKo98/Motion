/**
 * @file GenericEncoder.h
 * @brief Defines the hardware interface for quadrature rotary encoders.
 * @details This file contains the configuration structure and class definition for handling
 *          rotary encoders.
 */

#pragma once

#include "Core/IO/Sensor/BaseSensor.h"

namespace Motion::Core::IO {

/**
 * @brief Configuration parameters for the rotary encoder.
 * @details Holds the GPIO pin assignments for the quadrature signals.
 */
struct EncoderConfig {
    /**
     * @brief GPIO pin number for the Encoder Phase A signal.
     */
    uint8_t pinA;

    /**
     * @brief GPIO pin number for the Encoder Phase B signal.
     */
    uint8_t pinB;
};

/**
 * @brief A hardware interface for a quadrature rotary encoder.
 * @details This class provides an abstraction for reading rotary encoder values.
 *          It inherits from `BaseSensor<int32_t>`, providing raw count values as 32-bit integers.
 */
class GenericEncoder : public BaseSensor<int32_t> {
protected:
    /**
     * @brief Constructs a new GenericEncoder object.
     * @details Initializes the base sensor with a name and stores the encoder configuration.
     * @param name A human-readable unique identifier for this sensor (e.g., "Left Encoder").
     *             This is useful for logging and debugging purposes.
     * @param config A structure containing the Encoder configuration (pins).
     * @note This constructor does not initialize the hardware peripherals. Hardware setup is typically
     *       performed during a subsequent initialization phase.
     */
    explicit GenericEncoder(const char* name, EncoderConfig config) : BaseSensor<int32_t>(name), _encoderConfig(config) {};

    /**
     * @brief Internal storage for the encoder configuration.
     */
    EncoderConfig _encoderConfig;
};

/**
 * @relates GenericEncoder
 * @brief Smart handle for Encoder instances.
 */
using GenericEncoderHandle = SensorPointer(GenericEncoder);

} // namespace Motion::Core::IO