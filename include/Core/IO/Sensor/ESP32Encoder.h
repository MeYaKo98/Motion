/**
 * @file ESP32Encoder.h
 * @brief ESP32 hardware interface for rotary encoders.
 * @details Implements encoder reading using the ESP32 PCNT (Pulse Counter) peripheral.
 */

#pragma once

#include "Core/IO/Sensor/GenericEncoder.h"
#include "Arduino.h"
#include "driver/pcnt.h"
#include "driver/periph_ctrl.h"

namespace Motion::Core::IO {

/**
 * @brief Hardware interface for a rotary encoder on the ESP32 platform.
 * @details This class utilizes the ESP32's Pulse Counter (PCNT) hardware to count encoder ticks
 *          efficiently. It handles hardware resource allocation and interrupt-based overflow management
 *          to support continuous rotation.
 */
class ESP32Encoder : public GenericEncoder {
public:
    /**
     * @brief Constructs a new ESP32Encoder object.
     * @param name A human-readable unique identifier (e.g., "Left Encoder").
     * @param config A structure containing the Encoder configuration (pins, resolution, etc.).
     */
    ESP32Encoder(const char* name, EncoderConfig config);

    /**
     * @brief Destroys the ESP32Encoder object.
     * @details Releases any allocated PCNT hardware units and cleans up resources.
     */
    ~ESP32Encoder();

    /**
     * @brief Starts the encoder hardware interface.
     * @details Allocates a PCNT unit, configures GPIOs, and attaches interrupts.
     * @return true If the encoder started successfully.
     * @return false If the encoder failed to start (e.g., no PCNT units available).
     */
    bool Start() override;

    /**
     * @brief Stops the encoder hardware interface.
     * @details Disables the PCNT unit and releases resources.
     */
    void Stop() override;

protected:
    /**
     * @brief Returns the current encoder count in ticks.
     * @details Reads the hardware counter and combines it with the overflow counter.
     * @return int32_t The total number of ticks.
     */
    int32_t ReadSensor() override;

private:
    /**
     * @brief ISR handler to manage Encoder Saturation/Overflow.
     * @details Called when the PCNT counter overflows or underflows.
     * @param arg An address passed while attaching the ISR handler (the encoder instance in this case).
     * @note This function runs in IRAM context (ISR).
     */
    static void IRAM_ATTR isr_handler(void *arg);

    /**
     * @brief Counter for hardware timer overflows to extend range beyond 16-bit.
     * @todo add thread safety
     */
    int8_t _overflowCounter;

    /**
     * @brief Checks for a free PCNT unit and allocates it.
     * @return pcnt_unit_t The allocated PCNT unit, or an error indicator if none are free.
     */
    pcnt_unit_t AllocateUnit();

    /**
     * @brief Frees the allocated PCNT unit.
     * @param unit The PCNT unit to release.
     */
    void ReleaseUnit(pcnt_unit_t unit);

    /**
     * @brief Bitmask tracking allocated PCNT units.
     * @details Each bit corresponds to a PCNT unit index.
     */
    static uint8_t _usedUnitsMask;
    
    /**
     * @brief The PCNT unit instance used by this object.
     */
    pcnt_unit_t _pcntUnit;
};


} // namespace Motion::Core::IO