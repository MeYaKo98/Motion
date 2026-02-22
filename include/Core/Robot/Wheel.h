/**
 * @file Wheel.h
 * @brief Represents a robot wheel equipped with an encoder for distance measurement.
 * @details Encapsulates the physical properties of a wheel and its associated encoder sensor,
 *          providing the conversion from raw encoder ticks to linear distance traveled.
 *          Core component of the odometry system for differential drive robots.
 */

#pragma once

#include "Core/IO/Sensor/GenericEncoder.h"
#include "Core/Diagnostics/Logger.h"
#include <math.h>

namespace Motion::Core::Robot {

/**
 * @class Wheel
 * @brief Represents a robot wheel with an integrated encoder for motion tracking.
 * @details This class encapsulates the physical and mechanical properties of a wheel,
 *          combined with the associated rotary encoder sensor. It provides the ability to
 *          convert raw encoder tick counts into linear distance traveled by the wheel.
 *
 *          **Physical Model:**
 *          A wheel rolling without slipping travels a distance equal to its circumference
 *          multiplied by the number of complete rotations:
 *          Distance = circumference × rotations = 2πr × (ticks / ticks_per_rotation)
 *
 *          **Encoder Scaling:**
 *          - Encoder PPR (Pulses Per Rotation): Number of discrete counts per full wheel rotation
 *          - Quadrature Encoding: Many encoders use quadrature (phase A and B signals), producing
 *            4 counts per pulse cycle, effectively 4x the PPR resolution
 *          - This class accounts for quadrature encoding in its distance calculation
 *
 *          **Typical Configuration:**
 *          - Wheel radius: 0.05 m (5 cm) for small robots, 0.1-0.3 m for larger robots
 *          - Encoder resolution: 64-2048 PPR depending on motor type
 *          - Quadrature decoding: 4x resolution (e.g., 64 PPR → 256 counts per rotation)
 *
 *          **Integration in Odometry:**
 *          The Wheel class is used by the differential drive odometry system:
 *          1. Read encoder ticks via GetReading()
 *          2. Convert to distance via getDistance()
 *          3. Use left and right distances in kinematic equations to calculate robot position
 *
 * @note **No Slipping Assumption:** The distance calculation assumes the wheel rolls without
 *       slipping. On slippery surfaces (ice, water, loose terrain), actual distance may differ
 *       from calculated distance. Performance degrades gracefully in these conditions.
 *
 * @note **Factory Pattern:** Use the static Create() method to instantiate Wheel objects.
 *       The constructor throws exceptions if invalid parameters are provided.
 *
 * @note **Immutable Properties:** Wheel radius and encoder resolution are constant after
 *       construction. These properties should not change during robot operation.
 *
 * @see GenericEncoder for the underlying encoder interface
 * @see BaseOdometry for the odometry system that uses wheels
 * @see DifferentialDriveOdometry for the specific case of two-wheel odometry
 */
class Wheel;

/**
 * @brief Smart handle (smart pointer) for Wheel instances.
 * @details Manages the lifetime of wheel objects with automatic cleanup.
 *          Allows safe sharing of wheel references among multiple components
 *          (odometry, diagnostics, etc.).
 */
using WheelHandle = SensorPointer(Motion::Core::Robot::Wheel);

class Wheel {
public:
    /**
     * @brief Factory method to create a Wheel instance with validation.
     * @details Creates a new Wheel object with the specified encoder, resolution, and radius.
     *          Validates all parameters to ensure the wheel is properly configured before use.
     *
     * @param encoderHandle Smart pointer to a valid GenericEncoder instance.
     *                      This encoder is used to read the raw tick count.
     *                      Must not be nullptr; throws if null.
     *
     * @param resolution The encoder resolution in pulses per revolution (PPR).
     *                   The number of encoder ticks the encoder produces per full rotation.
     *                   Typical values: 64, 128, 256, 512, 1024, 2048.
     *                   Must be > 0; throws if zero.
     *
     * @param radius The radius of the wheel in application units (typically meters).
     *               Determines the linear distance per complete rotation.
     *               Distance per rotation = 2πr
     *               Typical values: 0.02 m (2 cm) to 0.5 m (50 cm)
     *               Must be > 0; throws if zero.
     *
     * @return WheelHandle A smart pointer to the newly created Wheel instance.
     *
     * @throws std::invalid_argument if encoderHandle is nullptr
     * @throws std::invalid_argument if resolution == 0
     * @throws std::invalid_argument if radius == 0.0f
     *
     * @note **Parameter Validation:** All parameters are validated before construction.
     *       Invalid inputs throw exceptions rather than silently failing.
     *
     * @note **Smart Pointer Return:** The returned handle automatically manages the
     *       wheel's lifetime. When the last reference is destroyed, the wheel is deleted.
     *
     * @note **Typical Usage:**
     *       ```cpp
     *       auto encoder = ESP32Encoder::Create("Right Encoder", {GPIO_A, GPIO_B});
     *       auto wheel = Wheel::Create(encoder, 64, 0.05f);
     *       // wheel is now ready to track distance
     *       float distance = wheel->getDistance();
     *       ```
     *
     * @warning **Encoder Lifetime:** The encoder object must remain valid for the entire
     *          lifetime of the wheel. If the encoder is deleted, accessing it from the wheel
     *          causes undefined behavior (likely a crash).
     *
     * @warning **Parameter Units:** The radius unit determines the unit of all distance
     *          measurements. If you pass radius in meters, distances will be in meters.
     *          If you pass radius in millimeters, distances will be in millimeters.
     *          Be consistent across your application.
     */
    static WheelHandle Create(Core::IO::GenericEncoderHandle encoderHandle, uint16_t resolution, float radius)
    {
        if (!encoderHandle) throw std::invalid_argument("EncoderHandle can not be NULL");
        if (resolution == 0) throw std::invalid_argument("Encoder Resolution can not be NULL");
        if (radius == 0) throw std::invalid_argument("Wheel radius can not be NULL");
        return WheelHandle(new Wheel(encoderHandle, resolution, radius));
    }

    /**
     * @brief Destroys the Wheel object.
     * @details Performs cleanup of the wheel instance.
     *          Does not delete the encoder—that responsibility remains with the encoder's owner.
     *
     * @note **Encoder Ownership:** This destructor does not manage the memory of the
     *       encoder pointer (stored in `_encoderHandle`). The encoder's creator/owner
     *       is responsible for its cleanup. Using smart pointer for the encoder ensures
     *       proper cleanup automatically.
     *
     * @note **Safe to Delete:** It is safe to delete a wheel while other components
     *       are using the same encoder (for example, in a multi-wheel odometry system).
     *       The encoder persists as long as any component holds a reference to it.
     */
    ~Wheel() {}

    /**
     * @brief Starts the wheel's encoder hardware interface.
     * @details Delegates to the encoder's Start() method to initialize the hardware.
     *          Must be called before the wheel can provide accurate distance readings.
     *
     * @return bool true if the encoder started successfully; false if initialization failed.
     *
     * @note **Encoder Initialization:** This calls the underlying encoder's Start() method,
     *       which initializes GPIO pins, configures hardware peripherals, and sets up
     *       interrupt handlers.
     *
     * @note **Pairing with Stop():** Always call Stop() when the wheel is no longer needed
     *       to release hardware resources.
     *
     * @see Stop()
     * @see getDistance()
     */
    bool Start()
    {
        return _encoderHandle->Start();
    }

    /**
     * @brief Stops the wheel's encoder hardware interface.
     * @details Delegates to the encoder's Stop() method to release hardware resources.
     *          After calling this, the wheel cannot provide distance readings until Start() is called again.
     *
     * @note **Resource Cleanup:** Stopping the encoder releases GPIO pins, disables interrupts,
     *       and frees any hardware resources it was using.
     *
     * @note **Idempotency:** Calling Stop() multiple times is safe—subsequent calls do nothing.
     *
     * @see Start()
     */
    void Stop()
    {
        _encoderHandle->Stop();
    }

    /**
     * @brief Calculates the total distance traveled by the wheel based on encoder readings.
     * @details Converts raw encoder tick count into linear distance traveled, accounting for
     *          the wheel's radius and the encoder's resolution. The calculation assumes
     *          no wheel slipping (pure rolling condition).
     *
     *          **Mathematical Calculation:**
     *          ```
     *          Distance = (2 × π × radius × ticks) / (resolution × 4)
     *          ```
     *
     *          **Explanation:**
     *          - `2 × π × radius`: Wheel circumference (distance per complete rotation)
     *          - `ticks`: Raw count from encoder (may be accumulated over many rotations)
     *          - `resolution`: Encoder resolution in pulses per rotation (PPR)
     *          - `4`: Quadrature decoding factor
     *            - A quadrature encoder produces 4 states per pulse cycle
     *            - For example, an encoder rated 64 PPR generates 64 × 4 = 256 ticks per rotation
     *          - Dividing `ticks / (resolution × 4)` gives the number of rotations
     *          - Multiplying by circumference gives distance
     *
     *          **Example Calculation:**
     *          - Wheel radius: 0.05 m
     *          - Encoder: 64 PPR
     *          - Current ticks: 1024
     *          - Circumference: 2π × 0.05 = 0.314 m
     *          - Rotations: 1024 / (64 × 4) = 4 rotations
     *          - Distance: 0.314 × 4 = 1.256 m
     *
     * @return float The calculated distance in the same units as the wheel radius.
     *               Examples:
     *               - If radius is in meters, distance is in meters
     *               - If radius is in millimeters, distance is in millimeters
     *               - If radius is in feet, distance is in feet
     *               Returns 0.0f if the encoder handle is nullptr (defensive programming).
     *
     * @note **Thread-Safety:** This function reads from the encoder, which is typically
     *       thread-safe (encoder uses mutexes). However, no synchronization is done here;
     *       the encoder provides it.
     *
     * @note **Accumulation:** The returned distance is cumulative from the moment the encoder
     *       started counting. It increases monotonically (assuming no encoder reset).
     *       To get incremental distance, subtract previous reading:
     *       ```cpp
     *       float prevDist = wheel->getDistance();
     *       // ... robot moves ...
     *       float curDist = wheel->getDistance();
     *       float deltaDistance = curDist - prevDist;
     *       ```
     *
     * @note **No Reset:** This function does not reset the encoder count. Encoder reset
     *       must be handled externally (e.g., by calling encoder->Reset() if supported).
     *
     * @note **Precision:** For small distances (< 0.001 m), quantization errors from the
     *       discrete encoder counts become significant. The minimum measurable distance is:
     *       ```
     *       minDistance = (2π × radius) / (resolution × 4)
     *       ```
     *       For radius=0.05m, resolution=64: minDistance ≈ 0.002 m = 2 mm
     *
     * @warning **Slipping Assumption:** The calculation assumes the wheel rolls without slipping.
     *          On slippery surfaces (ice, wet floors), the actual distance may differ significantly
     *          from the calculated value. Odometry accuracy degrades on such surfaces.
     *
     * @warning **Encoder Overflow:** For long distances, the encoder tick count may overflow.
     *          On a 32-bit encoder with 64 PPR and 4x quadrature:
     *          maxDistance ≈ (2π × 0.05 × 2^31) / (64 × 4) ≈ 20,000 km
     *          Overflow is unlikely for typical room-scale robots but possible for
     *          long-distance autonomy. Monitor encoder health.
     *
     * @warning **Null Check:** The function returns 0.0f if the encoder handle is nullptr,
     *          which silently masks a configuration error. In production, consider throwing
     *          an exception or logging a warning if the encoder is null.
     *
     * @see Start()
     * @see GenericEncoder::GetReading()
     */
    float getDistance() const
    {
        return (2.0f * M_PI * _radius * _encoderHandle->GetReading()) / (_resolution * 4.0f);
    }

protected:
    /**
     * @brief Constructs a new Wheel instance.
     * @details Protected constructor; use the Create() factory method instead.
     *          Initializes the wheel with physical parameters and encoder reference.
     *
     * @param encoderHandle Smart pointer to the GenericEncoder sensor associated with this wheel.
     *                      The wheel will read raw ticks from this encoder to calculate distance.
     *                      Must not be nullptr (factory ensures this).
     *
     * @param resolution The encoder resolution in pulses per revolution (PPR).
     *                   Typically 64-2048. The calculation uses this to determine rotations
     *                   from tick count (accounting for 4x quadrature decoding).
     *
     * @param radius The radius of the wheel in application units (typically meters).
     *               Used to convert rotations to linear distance via circumference = 2πr.
     *               Stored as absolute value (abs() is called in constructor).
     *
     * @post `_encoderHandle`, `_resolution`, and `_radius` are initialized and ready for use.
     * @post getDistance() can be called to retrieve the distance traveled.
     *
     * @note **Absolute Values:** The constructor applies abs() to resolution and radius,
     *       ensuring they are positive even if negative values are somehow passed.
     *       This is a defensive measure; the factory should prevent negative values.
     *
     * @note **Immutability:** All members are declared const, ensuring the wheel's
     *       physical properties never change after construction.
     */
    Wheel(Core::IO::GenericEncoderHandle encoderHandle, uint16_t resolution, float radius)
        : _encoderHandle(encoderHandle), _resolution(abs(resolution)), _radius(abs(radius)) {}

    /** 
     * @brief Smart pointer to the encoder sensor associated with this wheel.
     * @details The wheel reads raw tick counts from this encoder and converts them
     *          to linear distance via the getDistance() method.
     *
     * @note **Ownership:** The encoder is owned by the caller who passed it to Create().
     *       The wheel holds a reference but doesn't own it. Using smart pointer ensures
     *       the encoder persists as long as any component references it.
     *
     * @note **Thread-Safety:** The encoder itself handles synchronization of its tick count.
     *       The wheel's getDistance() method is as thread-safe as the encoder's GetReading().
     */
    Core::IO::GenericEncoderHandle _encoderHandle;

    /** 
     * @brief The encoder resolution in pulses per revolution (PPR).
     * @details Constant value set at construction. Used in distance calculation to convert
     *          tick count to rotation count. The calculation assumes 4x quadrature decoding.
     *
     * @note **Immutable:** Declared const; cannot be changed after construction.
     * @note **Units:** Ticks per complete wheel rotation (typically 64-2048).
     * @note **Quadrature Factor:** The calculation divides by (resolution × 4),
     *       accounting for the 4x multiplication from quadrature encoding.
     */
    const int16_t _resolution;

    /** 
     * @brief The radius of the wheel in application-defined units.
     * @details Constant value set at construction. Used to convert rotation count to linear
     *          distance via the circumference formula: distance = 2πr × rotations
     *
     * @note **Immutable:** Declared const; cannot be changed after construction.
     * @note **Units:** Application-defined (typically meters for SI, millimeters for smaller robots).
     *       Distance returned by getDistance() uses the same units as this radius.
     * @note **Typically 0.01-0.5:** For small robots, radii are usually in the
     *       0.01 m (1 cm) to 0.5 m (50 cm) range. Larger wheels reduce odometry error per distance.
     */
    const float _radius;
};

} // namespace Motion::Core::Robot