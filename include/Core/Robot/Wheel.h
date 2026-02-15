#pragma once

#include "Core/IO/Sensor/GenericEncoder.h"
#include "Core/Diagnostics/Logger.h"
#include <math.h>

namespace Motion::Core::Robot {

/**
 * @class Wheel
 * @brief Represents a robot wheel equipped with an encoder for distance measurement.
 *
 * @details This class encapsulates the physical properties of a wheel (radius) and its
 *          associated sensor (encoder resolution). It provides functionality to convert
 *          raw encoder readings into linear distance traveled.
 */
class Wheel;

/**
 * @relates Wheel
 * @brief Smart handle for Wheel instances.
 */
using WheelHandle = SensorPointer(Wheel);

class Wheel {
public:
    static WheelHandle Create(Core::IO::GenericEncoderHandle encoderHandle, uint16_t resolution, float radius)
    {
        if (!encoderHandle) throw std::invalid_argument("EncoderHandle can not be NULL");
        if (resolution == 0) throw std::invalid_argument("Encoder Resolution can not be NULL");
        if (radius == 0) throw std::invalid_argument("Wheel raidius can not be NULL");
        return WheelHandle(new Wheel(encoderHandle, resolution, radius));
    }

    /**
     * @brief Destroys the Wheel object.
     *
     * @note This destructor does not manage the memory of the `_encoder` pointer.
     *       The owner of the encoder object is responsible for its deletion.
     */
    ~Wheel() {}

    bool Start()
    {
        return _encoderHandle->Start();
    }

    void Stop()
    {
        _encoderHandle->Stop();
    }

    /**
     * @brief Calculates the total distance traveled by the wheel based on encoder readings.
     *
     * @details The distance is calculated using the formula:
     *          Distance = (2 * PI * radius * ticks) / (resolution * 4)
     *          The factor of 4 accounts for quadrature encoding where 4 counts are generated per pulse cycle.
     *
     * @return float The calculated distance in the same units as the wheel radius.
     *         Returns 0.0f if the internal encoder pointer is null.
     */
    float getDistance() const
    {
        return (2.0f * M_PI * _radius * _encoderHandle->GetReading()) / (_resolution * 4.0f);
    };

protected:
    /**
     * @brief Constructs a new Wheel instance.
     *
     * @param encoder Pointer to the generic encoder sensor associated with this wheel.
     * @param resolution The resolution of the encoder in pulses per revolution (PPR).
     *                   Note: The calculation assumes quadrature decoding (4x resolution).
     * @param radius The radius of the wheel. The unit of this value determines the unit
     *               of the calculated distance (e.g., meters).
     *
     * @warning The `encoder` pointer is stored internally. The caller is responsible for
     *          ensuring the lifetime of the `encoder` object exceeds the lifetime of this
     *          `Wheel` instance to avoid dangling pointers.
     */
    Wheel(Core::IO::GenericEncoderHandle encoderHandle, uint16_t resolution, float radius)
        : _encoderHandle(encoderHandle), _resolution(abs(resolution)), _radius(abs(radius)) {}

    /** @brief Pointer to the encoder sensor. */
    Core::IO::GenericEncoderHandle _encoderHandle;
    /** @brief The encoder resolution (pulses per revolution). */
    const int16_t _resolution;
    /** @brief The radius of the wheel. */
    const float _radius;
};

} // namespace Motion::Core::Robot