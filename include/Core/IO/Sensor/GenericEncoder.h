/**
 * @file GenericEncoder.h
 * @brief Defines the hardware interface for quadrature rotary encoders.
 * @details Provides configuration and base class for encoder implementations on various platforms.
 */

#pragma once

#include "Core/IO/Sensor/BaseSensor.h"

namespace Motion::Core::IO {

/**
 * @brief Configuration parameters for a rotary encoder.
 * @details Holds the GPIO pin assignments for the two-phase quadrature signals.
 *          A quadrature encoder provides two square-wave signals (Phase A and Phase B)
 *          that are 90 degrees out of phase, allowing detection of both direction and magnitude of rotation.
 *
 *          **Quadrature Encoding Basics:**
 *          - Phase A (pulse): Primary square-wave signal
 *          - Phase B (direction): Secondary signal 90° delayed
 *          - Forward rotation: A leads B (A edges occur before B transitions)
 *          - Reverse rotation: B leads A (B edges occur before A transitions)
 *          - Resolution: 4 counts per complete cycle (rising/falling on both channels)
 *
 *          **Typical Encoder Configurations:**
 *          - Typical 600 PPR (Pulses Per Revolution) encoder: 2400 counts per rev (600 × 4)
 *          - Motor-integrated encoder: 8-64 PPR common
 *          - High-precision encoders: 1024-10000 PPR available
 *
 * @note **Output Types:**
 *       - Open-drain: Requires pull-up resistors (typical for low-power encoders)
 *       - Push-pull: Active high/low output (higher current, faster response)
 *       - Differential: Complementary A/A-bar, B/B-bar (best noise immunity, rare in robotics)
 *       This configuration assumes single-ended open-drain or push-pull.
 *
 * @note **Pin Numbering:** Platform-specific (0-39 on ESP32, 0-13 on Arduino Uno, etc.).
 *       Validation of pin validity happens at runtime (Start() method).
 *
 * @see ESP32Encoder for an ESP32-specific implementation using PCNT
 * @see GenericEncoder for the base class
 */
struct EncoderConfig {
    /**
     * @brief GPIO pin number for the Encoder Phase A signal.
     * @details The primary encoder output line.
     *          Typically labeled A, Inc (increment), or Pulse on encoder datasheets.
     *          Generates a square wave proportional to rotation speed.
     *          Valid for any platform GPIO pin (validation at Start() time).
     *
     * @note **Phase A Properties:**
     *       - Frequency at max speed = (speed_rpm × PPR × 4) / 60
     *       - E.g., 300 RPM, 600 PPR encoder: 12,000 Hz max frequency
     *       - Rising edges mark unit increments (or 4× edges for quadrature)
     *
     * @note **Electrical:** Typically 3.3V or 5V logic levels. Confirm with encoder datasheet.
     *
     * @warning Must not equal pinB (encoder requires two distinct pins).
     */
    uint8_t pinA;

    /**
     * @brief GPIO pin number for the Encoder Phase B signal.
     * @details The secondary encoder output line (90° phase-shifted from A).
     *          Typically labeled B, Dir (direction), or Clock on encoder datasheets.
     *          Phase relationship to A determines rotation direction.
     *          Valid for any platform GPIO pin (validation at Start() time).
     *
     * @note **Phase B Properties:**
     *       - Same frequency as Phase A, but delayed 90° (quarter period)
     *       - Decoding: If A leads B → forward; if B leads A → reverse
     *       - For counting purposes: both rising and falling edges of both phases matter
     *       - Quadrature multiplication: single A pulse = 4 encoder counts (A↑, A↓, B↑, B↓)
     *
     * @note **Electrical:** Same logic levels as Phase A.
     *
     * @warning Must not equal pinA (encoder requires two distinct pins).
     */
    uint8_t pinB;
};

/**
 * @brief A hardware interface for a quadrature rotary encoder.
 * @details This class provides an abstraction for reading rotary encoder values.
 *          It inherits from `BaseSensor<int32_t>`, providing raw count values as 32-bit integers.
 *          Derived classes (ESP32Encoder, etc.) implement the actual hardware reading.
 *
 *          **Encoder Behavior:**
 *          - GetReading() returns the current accumulated tick count (int32_t)
 *          - Tick count increments for forward rotation, decrements for reverse
 *          - Tick = one complete quadrature cycle edge = 1/4 of a mechanical pulse
 *          - To get full mechanical pulses, divide by 4
 *
 *          **Applications:**
 *          - Wheel odometry: integral of velocity measured via encoder ticks
 *          - Motor feedback control: closed-loop speed/position regulation
 *          - Dead reckoning: estimate robot pose from wheel rotations
 *          - Slip detection: compare left/right wheel displacements
 *
 *          **Assumptions:**
 *          - Quadrature signals are clean and properly debounced
 *          - Phase A and Phase B are 90° out of phase
 *          - Encoder does not reverse direction during a single cycle (rare assumption)
 *          - Tick count accumulates linearly with rotation (no missed counts)
 *
 * @note **Reading Units:** int32_t ticks (range: -2.1 billion to +2.1 billion)
 *       For typical encoders and robots, count resets may never occur.
 *       A 600 PPR encoder at max speed (300 rpm) generates 12,000 ticks/sec
 *       (reaching int32_t max would take ~50,000 hours of continuous rotation).
 *
 * @note **Platform-Specific:** Derived implementations use platform-specific hardware
 *       (e.g., PCNT on ESP32, GPIO edge detection on other platforms) for efficiency.
 *
 * @note **Resolution:** 4× the mechanical PPR due to quadrature decoding.
 *       A 600 PPR encoder provides 2400 counts per revolution.
 *       E.g., wheel radius 0.05m → 0.314m circumference → 131 micrometers per count.
 *
 * @see ESP32Encoder for an ESP32-specific implementation
 * @see BaseSensor for the sensor base class abstraction
 */
class GenericEncoder : public BaseSensor<int32_t> {
protected:
    /**
     * @brief Constructs a new GenericEncoder object.
     * @details Protected constructor; derived classes use this to initialize the base sensor.
     *          Initializes the base sensor with the encoder name and configuration.
     *          Hardware is not initialized (call Start() when ready).
     *
     * @param name A human-readable unique identifier for this sensor (e.g., "Left Wheel Encoder").
     *             Used for logging, identification, and debugging.
     *             String must have static or long-lived scope (not stack-allocated).
     *
     * @param config EncoderConfig structure containing:
     *               - pinA: GPIO pin for Phase A signal
     *               - pinB: GPIO pin for Phase B signal
     *               Stored internally for later use by derived implementations.
     *
     * @post The encoder is initialized with:
     *       - Name stored in ISensor._name
     *       - Type set to "int32_t" (via BaseSensor<int32_t>)
     *       - Type size set to sizeof(int32_t) = 4 bytes
     *       - Configuration stored in `_encoderConfig`
     *       - Hardware is NOT yet initialized (call Start())
     *
     * @note **Protected Constructor:** Prevents direct instantiation. Derived concrete classes
     *       (ESP32Encoder, etc.) call this in their constructors.
     *
     * @note **Configuration Storage:** The EncoderConfig is stored as a member, not copied
     *       at runtime. Derived implementations access it via `_encoderConfig`.
     *
     * @note **Initialization Sequence:** BaseSensor calls ISensor(name, type, typeSize),
     *       where type = "int32_t" and typeSize = 4. This embeds type information in metadata.
     *
     * @warning The name pointer is not copied; ensure string scope exceeds the encoder's lifetime.
     *          Use string literals: `GenericEncoder("Left Wheel")` ✓
     *          Avoid stack strings: `sprintf(buf, "Wheel%d"); GenericEncoder(buf)` ✗
     *
     * @warning The config reference is stored. Ensure the structure remains valid,
     *          or copy it in derived classes if needed.
     */
    explicit GenericEncoder(const char* name, EncoderConfig config) : BaseSensor<int32_t>(name), _encoderConfig(config) {};

    /**
     * @brief Internal storage for the encoder configuration.
     * @details Holds the GPIO pin assignments (pinA, pinB).
     *          Accessed by derived implementations to configure hardware.
     * @note Invariant: pins don't change after construction.
     */
    EncoderConfig _encoderConfig;
};

/**
 * @brief Smart handle for Encoder instances.
 * @details Uses smart pointers for automatic lifetime management and reference counting.
 *          Recommended for ownership and passing encoders through the application.
 */
using GenericEncoderHandle = SensorPointer(Motion::Core::IO::GenericEncoder);

} // namespace Motion::Core::IO