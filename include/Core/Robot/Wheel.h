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
class Wheel {
public:
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
    Wheel(IO::GenericEncoder* encoder, int16_t resolution, float radius)
        : _encoder(encoder), _resolution(abs(resolution)), _radius(abs(radius)) {
            if (_encoder == nullptr) LOG_ERROR("Encoder can not be NULL");
            if (_resolution == 0) LOG_ERROR("Resolution cannot be zero");
            if (_radius == 0.0) LOG_ERROR("Radius cannot be zero");
        }

    /**
     * @brief Destroys the Wheel object.
     *
     * @note This destructor does not manage the memory of the `_encoder` pointer.
     *       The owner of the encoder object is responsible for its deletion.
     */
    ~Wheel() {}

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
    float getDistance() const {
        if (!_encoder) return 0.0f;
        return (2.0f * M_PI * _radius * _encoder->GetReading()) / (_resolution * 4.0f);
    };

private:
    /** @brief Pointer to the encoder sensor. */
    IO::GenericEncoder* _encoder;
    /** @brief The encoder resolution (pulses per revolution). */
    const int16_t _resolution;
    /** @brief The radius of the wheel. */
    const float _radius;
};

} // namespace Motion::Core::Robot