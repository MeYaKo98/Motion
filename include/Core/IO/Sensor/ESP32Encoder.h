/**
 * @file ESP32Encoder.h
 * @brief ESP32 hardware interface for rotary encoders using PCNT peripheral.
 * @details Implements high-speed, low-latency encoder reading using the ESP32 Pulse Counter (PCNT).
 */

#pragma once

#include "Core/IO/Sensor/GenericEncoder.h"
#include "Arduino.h"
#include "driver/pcnt.h"
#include "driver/periph_ctrl.h"
#include <atomic>

namespace Motion::Core::IO {

/**
 * @brief Hardware interface for a rotary encoder on the ESP32 platform.
 * @details This class utilizes the ESP32's Pulse Counter (PCNT) hardware peripheral to count
 *          encoder ticks efficiently and with minimal CPU load. The PCNT hardware provides:
 *          - Quadrature decoding (2-phase encoding, 4x tick count)
 *          - Hardware-level counting (no software polling required)
 *          - High-speed counting (up to 40 MHz input frequency)
 *          - Overflow/underflow interrupts for 32-bit extended range
 *
 *          **PCNT Hardware Architecture:**
 *          - 8 independent PCNT units (0-7), each with 2 channels
 *          - 16-bit signed counter (-32768 to +32767)
 *          - Configurable counting edge (rising, falling, both)
 *          - Control pin for direction control (forward, reverse, latch)
 *          - High/low limit interrupts and event detection
 *
 *          **Quadrature Encoding:**
 *          - Phase A (pulse) and Phase B (control) pins on the encoder
 *          - 4 edges per complete cycle → 4x tick count
 *          - Forward rotation: A↑ B0, A↑ B1, A↓ B1, A↓ B0, repeat
 *          - Reverse rotation: Same sequence reversed
 *          - PCNT decodes automatically using the control pin logic
 *
 *          **Overflow Handling:**
 *          - 16-bit counter overflows at ±32768
 *          - ISR increments/decrements a software overflow counter
 *          - GetReading() combines: (overflow_counter × 32768) + hw_counter
 *          - Effectively extends range to full 32-bit (±2.1 billion ticks)
 *
 *          **Typical Usage:**
 *          ```cpp
 *          EncoderConfig cfg = {.pinA = 12, .pinB = 13};
 *          auto encoder = ESP32Encoder::Create("Left Wheel", cfg);
 *          if (encoder->Start()) {
 *              int32_t ticks = encoder->GetReading();  // Read accumulated ticks
 *              float distance = ticks * (2*PI*radius) / (resolution*4);  // Convert to distance
 *          }
 *          encoder->Stop();
 *          ```
 *
 * @note **Resource Sharing:** PCNT units are a finite resource (8 units for entire system).
 *       This class allocates one PCNT unit per encoder instance. Limit encoders to 8 per ESP32,
 *       or implement a shared/multiplexed PCNT manager.
 *
 * @note **ISR Usage:** This class uses an interrupt handler for overflow events.
 *       Overflow interrupts trigger ~once every 8.2 revolutions (1 MHz quadrature input).
 *       ISR execution is minimal (atomic counter increment); safe for real-time tasks.
 *
 * @warning **Pin Validation:** pinA and pinB must be different and valid GPIO numbers.
 *          The factory validates this. Invalid pins cause Start() to fail silently.
 *
 * @warning **Quadrature Signal Quality:** PCNT expects clean, debounced quadrature signals.
 *          Noisy encoder signals cause missed counts. Use hardware debouncing or quality encoders.
 *
 * @see GenericEncoder for the base interface
 * @see BaseS ensor for the sensor base class
 */
class ESP32Encoder;

/**
 * @relates ESP32Encoder
 * @brief Smart handle for ESP32 Encoder instances.
 * @details Uses shared_ptr for automatic lifetime management.
 */
using ESP32EncoderHandle = SensorPointer(ESP32Encoder);

class ESP32Encoder : public GenericEncoder {
public:

    /**
     * @brief Factory method to create an ESP32Encoder instance.
     * @details Creates and validates a new encoder with the specified GPIO pin configuration.
     *
     * @param name Human-readable identifier (e.g., "Left Wheel Encoder").
     *             Used for logging and identification.
     *
     * @param config EncoderConfig with:
     *               - pinA: GPIO for Phase A signal (quadrature pulse)
     *               - pinB: GPIO for Phase B signal (direction control)
     *               Must satisfy: pinA != pinB
     *
     * @return ESP32EncoderHandle Shared pointer to the encoder instance.
     *
     * @throws std::invalid_argument if config.pinA == config.pinB
     *
     * @note **PCNT Unit Allocation:** The factory automatically allocates an available PCNT unit.
     *       If all 8 PCNT units are in use, allocation fails silently; Start() will return false.
     *
     * @warning Only 8 ESP32Encoder instances can coexist (one per PCNT unit).
     *          Creating a 9th encoder will fail in Start().
     *
     * @post The encoder is created but NOT started. Call Start() to initialize hardware.
     */
    static ESP32EncoderHandle Create(const char* name, EncoderConfig config);

    /**
     * @brief Destroys the ESP32Encoder object.
     * @details Ensures that the PCNT unit is released and interrupts are disabled.
     *          Calls Stop() to perform cleanup.
     *
     * @note The destructor automatically calls Stop(), so explicit Stop() is optional.
     */
    ~ESP32Encoder();

    /**
     * @brief Starts the encoder hardware interface.
     * @details Configures the allocated PCNT unit with the GPIO pins and quadrature decoding logic.
     *          Enables overflow/underflow interrupts and starts the hardware counter.
     *
     *          **Initialization Steps:**
     *          1. Configure pinA and pinB as INPUT_PULLUP (debounce via pull-ups)
     *          2. Configure PCNT unit with quadrature settings:
     *             - pinA: Primary pulse input
     *             - pinB: Control pin for direction discrimination
     *             - Up-count on leading edge, down-count on trailing edge
     *          3. Set soft limits at ±32767 to trigger overflow interrupts
     *          4. Attach ISR handler for overflow/underflow events
     *          5. Clear counter and start counting
     *
     * @return true if the encoder initialized successfully and is ready to count.
     * @return false if initialization failed (e.g., PCNT unit allocation failed, invalid pins).
     *
     * @post On success:
     *       - PCNT hardware is counting encoder edges
     *       - GetReading() will return accumulated tick count
     *       - Overflow/underflow interrupts are active
     *
     * @post On failure:
     *       - Encoder is non-functional
     *       - GetReading() returns 0 or invalid data
     *       - PCNT unit may not be available
     *
     * @note **Pull-Up Configuration:** The method sets INPUT_PULLUP on both pins.
     *       This works well for most encoders (they provide open-drain outputs).
     *       If your encoder is push-pull, disable the pull-ups in hardware.
     *
     * @note **ISR Service Installation:** The first encoder's Start() installs the PCNT ISR
     *       service globally. Subsequent encoders reuse the same ISR. This is a one-time cost.
     *
     * @note **Synchronous Behavior:** Start() is blocking; all hardware initialization
     *       completes before return.
     *
     * @warning **Double Start:** Calling Start() twice without Stop() may cause issues
     *          (double ISR registration). Call Stop() between Start() calls.
     *
     * @warning **Pin Use Validation:** Start() does NOT verify that pins are not in use elsewhere.
     *          Ensure no other peripheral (I2C, SPI, UART, GPIO) uses these pins.
     *
     * @warning **PCNT Unit Exhaustion:** If all 8 PCNT units are allocated, Start() fails.
     *          Limit encoders to 8 per ESP32, or implement dynamic unit sharing.
     *
     * @see Stop()
     * @see GetReading()
     */
    bool Start() override;

    /**
     * @brief Stops the encoder hardware interface.
     * @details Disables the PCNT counter, removes the ISR handler, and releases the PCNT unit.
     *          After Stop(), the encoder is non-functional until Start() is called again.
     *
     * @return void
     *
     * @post The PCNT unit is released:
     *       - Counter is paused
     *       - ISR is unregistered
     *       - PCNT unit becomes available for reuse
     *       - GetReading() will return stale data
     *
     * @note **Idempotency:** Calling Stop() multiple times is safe.
     *
     * @note **Resource Release:** Stop() is critical to release the PCNT unit
     *       for use by other encoders or applications.
     *
     * @note **Called by Destructor:** The destructor automatically calls Stop().
     *
     * @see Start()
     */
    void Stop() override;

protected:
    /**
     * @brief Constructs a new ESP32Encoder object.
     * @details Protected constructor; use Create() factory instead.
     *          Allocates a PCNT unit and initializes the encoder state.
     *
     * @param name Human-readable encoder name.
     * @param config GPIO pin configuration (validated by factory).
     *
     * @post The encoder is initialized with an allocated PCNT unit.
     *       Hardware is NOT yet configured; call Start() when ready.
     */
    explicit ESP32Encoder(const char* name, EncoderConfig config);

    /**
     * @brief Returns the current encoder count in ticks.
     * @details Reads the hardware counter (signed 16-bit) and combines it with the software
     *          overflow counter to produce a 32-bit signed tick count.
     *
     *          **Tick Calculation:**
     *          int32_t result = (overflow_counter × 32768) + hardware_counter;
     *          E.g., overflow_counter=2, hardware=1000 → result = 66536
     *
     * @return int32_t The total accumulated encoder ticks (range: ±2.1 billion).
     *                  Positive values: forward/clockwise rotation
     *                  Negative values: reverse/counter-clockwise rotation
     *                  Zero: no rotation since reset (or exact multiple of 2.1B rotations)
     *
     * @note **Reading Atomicity:** Snapshot of hardware counter and atomic overflow counter
     *       are read, but not fully synchronized. In rare cases (overflow at read time),
     *       a count may be off by ±32768. Acceptable for most robotics applications.
     *
     * @note **Quadrature Multiplication:** Hardware counts edges; 4 edges/rev for quadrature.
     *       To convert to wheel revolutions: revs = ticks / 4.
     *       To convert to distance: distance = ticks × (wheel_perimeter) / (pulses_per_rev × 4).
     *
     * @note **No Latency:** Reading is O(1) and takes microseconds; safe for high-frequency
     *       control loops (1000+ Hz).
     *
     * @warning **Overflow Counter Rollover:** The software overflow counter (int16_t) overflows
     *          after ±32768 increments. For a 1 MHz quadrature input (250 kHz ticks),
     *          overflow occurs every ~131 seconds. For typical wheels, this is millions of
     *          rotations and unlikely to occur in normal operation. If needed, implement
     *          full 32-bit overflow counter (use int32_t for _overflowCounter).
     *
     * @see BaseSensor::GetReading()
     */
    int32_t ReadSensor() override;

private:
    /**
     * @brief ISR handler to manage Encoder Overflow/Underflow.
     * @details Called by the PCNT peripheral when the hardware counter hits ±32767 limit.
     *          Increments or decrements the software overflow counter accordingly to extend
     *          the counter range beyond 16 bits.
     *
     * @param arg Pointer to the ESP32Encoder instance (passed during ISR registration).
     *
     * @note **IRAM Context:** This function runs in ISR context (interrupt handler).
     *       It must be minimal and fast (microseconds). IRAM_ATTR places code in fast memory.
     *
     * @note **Atomic Operation:** Overflow counter is std::atomic<int16_t>, so increment/decrement
     *       is atomic and thread-safe without explicit locking.
     *
     * @note **Frequency:** Overflow occurs approximately once every 8.2 rotations at 1 MHz input
     *       (250 kHz base tick rate). For a moving robot, overflows occur every second or so.
     *       ISR overhead is negligible.
     *
     * @warning **ISR Safety:** Avoid calling blocking operations or complex logic in the ISR.
     *          Only atomic operations are used; safe for FreeRTOS tasks waiting on data.
     */
    static void IRAM_ATTR isr_handler(void *arg);

    /**
     * @brief Counter for hardware timer overflows to extend range beyond 16-bit.
     * @details Incremented by ISR on high limit, decremented on low limit.
     *          Combined with hardware counter in ReadSensor() to form 32-bit result.
     * @note Uses std::atomic for thread-safe access without explicit locking.
     */
    std::atomic<int16_t> _overflowCounter;

    /**
     * @brief Checks for a free PCNT unit and allocates it.
     * @details Searches the bitmask of used PCNT units and returns the first free one.
     *          Also marks the unit as "in use" in the bitmask.
     *
     * @return pcnt_unit_t The allocated PCNT unit index (0-7).
     *                     If no units available, returns PCNT_UNIT_MAX (8).
     *
     * @note **Global Bitmask:** Uses a static class variable `_usedUnitsMask` shared across
     *       all ESP32Encoder instances to track allocation globally.
     *
     * @note **Thread-safety:** AllocateUnit() is NOT thread-safe. If multiple encoders
     *       are created simultaneously in different tasks, race conditions may occur.
     *       Call from a single initialization task or add a mutex.
     */
    pcnt_unit_t AllocateUnit();

    /**
     * @brief Frees a previously allocated PCNT unit.
     * @details Clears the bit in the global bitmask, marking the unit available for reuse.
     *
     * @param unit The PCNT unit to release (must have been allocated by AllocateUnit).
     *
     * @note **Idempotency:** Freeing an already-free unit is safe (clears a 0 bit).
     *
     * @note **Thread-safety:** NOT thread-safe. Ensure no concurrent alloc/free operations.
     */
    void ReleaseUnit(pcnt_unit_t unit);

    /**
     * @brief Bitmask tracking allocated PCNT units globally.
     * @details Each bit corresponds to a PCNT unit index (0=UNIT_0, 1=UNIT_1, etc.).
     *          Bit=1 means unit is allocated; bit=0 means free.
     * @note Static member shared across all ESP32Encoder instances.
     */
    static uint8_t _usedUnitsMask;

    /**
     * @brief The allocated PCNT unit for this encoder.
     * @details Set by AllocateUnit() during construction.
     *          If allocation failed, set to PCNT_UNIT_MAX.
     */
    pcnt_unit_t _pcntUnit;
};

} // namespace Motion::Core::IO