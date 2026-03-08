/**
 * @file ESP32DCMotor.h
 * @brief ESP32 hardware interface implementation for DC motors.
 * @details Provides the concrete ESP32-specific implementation for controlling DC motors
 *          using GPIO pins and PWM peripheral.
 */

#pragma once

#include "Motion/Core/IO/Actuator/GenericDCMotor.h"
#include "Arduino.h"

namespace Motion::Core::IO {

class ESP32DCMotor;

/**
 * @brief Smart handle for ESP32 Motor instances.
 * @details Uses smart pointers for automatic lifetime management and thread-safe reference counting.
 */
using ESP32DCMotorHandle = ActuatorPointer(Motion::Core::IO::ESP32DCMotor);


/**
 * @brief An ESP32 hardware interface for DC motor control.
 * @details This class implements the GenericDCMotor interface using ESP32-specific hardware features:
 *          - GPIO pins configured as OUTPUT
 *          - analogWrite() function for PWM duty cycle control
 *          - H-Bridge or dual-relay motor driver connected to two GPIO pins
 *
 *          **Motor Control Mechanism:**
 *          A DC motor requires two pins to control direction and speed:
 *          - Pin A (pinA): Forward direction or PWM for one winding
 *          - Pin B (pinB): Reverse direction or PWM for other winding
 *
 *          **Typical Wiring (H-Bridge Motor Driver):**
 *          - pinA voltage = forward speed (0-255 PWM)
 *          - pinB voltage = reverse speed (0-255 PWM)
 *          - Positive command → pinA high, pinB low (forward)
 *          - Negative command → pinB high, pinA low (reverse)
 *          - Zero command → both pins low (brake/coast)
 *
 *          **PWM Conversion:**
 *          The command float value (-1 to 1) is converted to PWM duty cycle (0-255):
 *          - Command +1.0 → PWM 255 (full forward)
 *          - Command +0.5 → PWM 127.5 ≈ 128 (half-speed forward)
 *          - Command   0 → PWM 0 (stopped)
 *          - Command -0.5 → PWM 127.5 ≈ 128 reverse (half-speed backward)
 *          - Command -1.0 → PWM 255 reverse (full backward)
 *
 * @note **ESP32 PWM Capabilities:**
 *       - 16 PWM channels available via LEDC peripheral
 *       - Frequency configurable (typically 5 kHz for motors)
 *       - 8-bit resolution (0-255) standard
 *       - All GPIO pins capable of PWM output
 *
 * @warning **Pin Configuration:** Both pinA and pinB must be:
 *          - Valid ESP32 GPIO pins (0-39, avoiding strapping pins)
 *          - Not in use by other peripherals
 *          - Connected to motor driver inputs (not directly to motor)
 *
 * @warning **Motor Driver Required:** Do not connect GPIO pins directly to the motor.
 *          Use an H-Bridge motor driver (e.g., L298N) to:
 *          - Provide sufficient current (GPIO can only provide ~40mA)
 *          - Protect GPIO from back-EMF when motor stops
 *          - Allow bidirectional control
 *
 * @see GenericDCMotor for the base interface
 * @see GenericMotor for the generic motor abstraction
 * @see BaseActuator for the actuator base class
 */
class ESP32DCMotor : public GenericDCMotor {
public:
    /**
     * @brief Factory method to create an ESP32DCMotor instance.
     * @details Creates and validates a new DC motor instance with the specified GPIO pin configuration.
     *          Performs input validation before instance creation.
     *
     * @param name A human-readable identifier for the motor (e.g., "Left Motor", "Right Wheel").
     *             Used for logging and telemetry. String must be statically allocated or have
     *             long-lived scope (not stack-allocated).
     *
     * @param config A DCMotorConfig structure containing:
     *               - pinA: GPIO pin for forward/primary control
     *               - pinB: GPIO pin for reverse/secondary control
     *               Must satisfy: pinA != pinB (two different pins required)
     *
     * @return ESP32DCMotorHandle A smart pointer to the newly created motor instance.
     *
     * @throws std::invalid_argument if config.pinA == config.pinB
     *         (cannot control motor with only one pin)
     *
     * @note **Pin Validation:** The factory validates that pins are different but does NOT check:
     *       - Pin validity (0-39 on ESP32)
     *       - Pin availability (whether in use by other peripherals)
     *       - Physical connection (Start() may still fail if pins are invalid)
     *
     * @note **Usage Pattern:**
     *       ```cpp
     *       DCMotorConfig cfg = {.pinA = 12, .pinB = 13};
     *       auto motor = ESP32DCMotor::Create("Left Motor", cfg);
     *       motor->Start(); // Initialize hardware
     *       motor->SetCommand(0.5f); // 50% forward speed
     *       ```
     *
     * @warning Always check the return value (Handle is never null from Create() if validation passes).
     */
    static ESP32DCMotorHandle Create(const char* name, DCMotorConfig config);

    /**
     * @brief Destroys the ESP32DCMotor object.
     * @details Ensures the motor is stopped and releases any hardware resources associated with it.
     *          Calls Stop() to halt the motor and clean up GPIO configurations.
     *
     * @note The destructor calls Stop() automatically, so manual Stop() before destruction is optional
     *       but recommended for explicit control.
     */
    ~ESP32DCMotor();

    /**
     * @brief Initializes and starts the DC motor hardware interface.
     * @details Configures the GPIO pins as OUTPUT mode and prepares the motor for commands.
     *          Must be called before SetCommand() or any motor operation.
     *
     *          **Initialization Steps:**
     *          1. Configure pinA as OUTPUT
     *          2. Configure pinB as OUTPUT
     *          3. Set both pins to LOW (motor off)
     *          4. Mark motor as started
     *
     * @return true
     *
     * @post On success:
     *       - Both GPIO pins are configured as outputs
     *       - Motor is in stopped state (both pins LOW)
     *       - SetCommand() will accept commands
     *       - The internal _started flag is set to true
     *
     * @post On failure:
     *       - GPIO state is undefined; State may be partially configured
     *       - Motor control is non-functional
     *
     * @note **Idempotency:** If Start() is called twice without Stop(), the second call
     *       reconfigures the pins (harmless) and returns success.
     *
     * @note **GPIO Locking:** Modern ESP-IDF automatically manages GPIO to prevent conflicts,
     *       but conflicts may still occur. Ensure no other subsystem uses these pins.
     *
     * @note **Power Supply:** Ensure the motor driver is powered before or immediately after Start().
     *       Without power, the motor driver won't respond to GPIO commands.
     *
     * @warning **Motor Jerks:** At Start(), the GPIO pins are LOW, which may cause a motor twitch
     *          if there's residual capacitive charge or stored command state. Be aware when starting
     *          motors in critical situations (e.g., suspended payload).
     *
     * @warning **No PWM Frequency Configuration:** This implementation uses Arduino's default PWM
     *          frequency (~5 kHz). Changing frequency requires low-level LEDC API calls.
     *
     * @see Stop()
     * @see SendCommand()
     */
    bool Start() override;

    /**
     * @brief Sends a control command to the DC motor.
     * @details Translates the abstract normalized command value into hardware-specific signals
     *          (PWM duty cycles on the two control pins). Implements direction and speed control
     *          via H-Bridge logic.
     *
     *          **Command Mapping:**
     *          Input range: [-1.0, +1.0] (normalized)
     *          - Positive (+0.5 to +1.0): Forward motion at increasing speeds
     *          - Zero (0.0): Motor stopped
     *          - Negative (-1.0 to -0.5): Backward motion at increasing speeds
     *
     *          **PWM Mapping Algorithm:**
     *          ```
     *          if (command > 0) {
     *              clamp command to [0, 1]
     *              pinA_PWM = command * 255
     *              pinB_PWM = 0  // disable reverse
     *          } else {
     *              clamp command to [-1, 0]
     *              pinB_PWM = -command * 255
     *              pinA_PWM = 0  // disable forward
     *          }
     *          ```
     *
     * @param command The control value to apply to the motor.
     *                Normalized float range: [-1.0, +1.0]
     *                - +1.0: Full forward speed
     *                - +0.5: Half forward speed
     *                -  0.0: Stop (both pins low, coast mode)
     *                - -0.5: Half backward speed
     *                - -1.0: Full backward speed
     *
     *                Values outside [-1, +1] are clamped to this range.
     *
     * @pre Start() must have been called successfully before this method.
     *      Calling SendCommand() before Start() has undefined behavior.
     *
     * @post The motor driver receives updated PWM signals on the configured pins.
     *       Physical motor motion may have a slight delay due to motor inertia.
     *
     * @note **Immediate Effect:** This method immediately updates the GPIO pins, so the motor
     *       responds within microseconds. No buffering or queueing occurs.
     *
     * @note **PWM Accuracy:** The PWM conversion uses integer arithmetic (command * 255),
     *       which may lose fractional precision for small command values. The motor's
     *       mechanical inertia typically masks this quantization.
     *
     * @note **Command Persistence:** The command is applied immediately and persists until
     *       the next call to SendCommand(). Call repeatedly at control frequency (typically 100+ Hz).
     *
     * @note **No Command Clamping at Base:** BaseActuator::SetCommand() handles clamp limits.
     *       This implementation (SendCommand) receives pre-validated command in [-1, 1] but
     *       re-clamps defensively.
     *
     * @note **Coast vs. Brake:** Setting command = 0 results in "coast" mode (both pins low, motor coasts).
     *       Real brake (both pins high) requires motor driver support and isn't implemented here.
     *
     * @note **Direction Reversal:** If the motor rotates opposite to expected, either:
     *       - Swap pinA and pinB in the config
     *       - Negate the command value before calling SetCommand()
     *       Document the chosen convention in your robot's setup code.
     *
     * @note **Thread-safety:** This method is NOT internally thread-safe. Concurrent calls from
     *       multiple tasks will cause race conditions on GPIO writes. Use external serialization
     *       (e.g., mutex) if calling from multiple FreeRTOS tasks.
     *
     * @warning **GPIO Output Only:** This implementation assumes pins are configured as outputs
     *          (done by Start()). If pins are reconfigured as inputs elsewhere, motor control fails.
     *
     * @warning **Command Saturation:** If the controller (BaseController) produces a command > 1.0
     *          (due to tuning issues), it's clamped here to ±1.0. The clamping happens silently.
     *          Monitor controller output in tuning/debugging to catch this.
     *
     * @warning **Motor Chatter at Zero:** Due to sensor noise and quantization, the controller may
     *          issue very small alternating commands (±0.01). This causes motor chatter (rapid
     *          forward-backward micro-motions). The motor's friction and inertia usually suppress this,
     *          but severe cases benefit from a dead-band filter on the error input.
     *
     * @see SetCommand() (inherited from BaseActuator)
     * @see Stop()
     */
    void SendCommand(float command) override;

    /**
     * @brief Stops the DC motor immediately.
     * @details Sets both control pins to LOW, halting all motor motion.
     *          This is a "coast" stop—the motor coasts to a halt due to friction.
     *
     *          **Stop Behavior:**
     *          - Both pinA and pinB set to digital LOW (0V)
     *          - Motor driver enters neutral state
     *          - Motor rotates freely (coasts to stop, not electrically braked)
     *          - Pending commands are not executed
     *
     * @pre Start() should have been called before Stop() (though Stop() is safe even if not).
     *
     * @post Motor is in stopped/coast state.
     *       GPIO pins remain configured as outputs (can command again via SetCommand()).
     *       No hardware resources are released (hardware is still initialized).
     *
     * @note **Coasting:** This implementation uses coast mode (both pins LOW).
     *       Some motor drivers support brake mode (both pins HIGH) for faster stopping,
     *       but it's not implemented here. Implement if needed via custom motor drivers.
     *
     * @note **Electrical Behavior:** The motor windings are disconnected (neutral state),
     *       allowing the rotor to spin freely. Magnetic cogging may cause small back-EMF
     *       depending on the motor design.
     *
     * @note **Called by Destructor:** The destructor automatically calls Stop(), so explicit
     *       Stop() before object destruction is optional (but recommended for clarity).
     *
     * @note **Idempotency:** Calling Stop() multiple times is safe and has the same effect
     *       as calling it once (both pins are already LOW).
     *
     * @note **Restart:** After Stop(), call SetCommand() again to resume operation.
     *       Call Start() only if hardware re-initialization is needed; SetCommand() works
     *       directly after Stop().
     *
     * @see Start()
     * @see SendCommand()
     */
    void Stop() override;

protected:
    /**
     * @brief Constructs a new ESP32DCMotor object.
     * @details Protected constructor; use the Create() factory method instead.
     *          Initializes the motor instance with a name and hardware configuration.
     *
     * @param name A human-readable name for the motor (e.g., "Left Motor").
     *             Used for identification and logging. String must be statically allocated
     *             or have long-lived scope.
     *
     * @param config The configuration structure containing GPIO pin assignments:
     *               - pinA: Primary control pin (forward or PWM phase 1)
     *               - pinB: Secondary control pin (reverse or PWM phase 2)
     *
     * @post The motor instance is initialized with the configuration.
     *       Hardware is NOT yet initialized; call Start() to initialize.
     *
     * @note This constructor is typically called by the Create() factory after validation.
     *
     * @warning The name and config are stored by reference (pointers); ensure their
     *          lifetime extends beyond this object's lifetime.
     */
    explicit ESP32DCMotor(const char* name, DCMotorConfig config);
};

} // namespace Motion::Core::IO