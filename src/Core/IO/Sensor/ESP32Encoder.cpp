/**
 * @file ESP32Encoder.cpp
 * @brief Encoder hardware interface for esp32.
 */

#include "Core\IO\Sensor\ESP32Encoder.h"

namespace Motion::Core::IO {

uint8_t ESP32Encoder::_usedUnitsMask = 0;

ESP32Encoder::ESP32Encoder(const char* name, EncoderConfig config)
    : GenericEncoder(name, config), _overflowCounter(0), _pcntUnit(AllocateUnit()) {}

ESP32Encoder::~ESP32Encoder() {
    ReleaseUnit(_pcntUnit);
}

pcnt_unit_t ESP32Encoder::AllocateUnit() {
    for (int i = 0; i < PCNT_UNIT_MAX; i++) {
        if (!(_usedUnitsMask & (1 << i))) { // Check if bit i is 0
            _usedUnitsMask |= (1 << i);     // Mark as used
            return (pcnt_unit_t)i;
        }
    }
    return PCNT_UNIT_MAX; // No units left
}

void ESP32Encoder::ReleaseUnit(pcnt_unit_t unit) {
    if (unit < PCNT_UNIT_MAX) {
        _usedUnitsMask &= ~(1 << unit); // Clear the bit
    }
}

bool ESP32Encoder::Start() {
    if (_pcntUnit >= PCNT_UNIT_MAX)
    {
        return false;
    }
    pinMode(this->_encoderConfig.pinA, INPUT_PULLUP);
    pinMode(this->_encoderConfig.pinB, INPUT_PULLUP);
    
    pcnt_config_t pcnt_ch0 = {
        .pulse_gpio_num  = this->_encoderConfig.pinA,
        .ctrl_gpio_num   = this->_encoderConfig.pinB,
        .lctrl_mode      = PCNT_MODE_REVERSE,
        .hctrl_mode      = PCNT_MODE_KEEP,
        .pos_mode        = PCNT_COUNT_INC,
        .neg_mode        = PCNT_COUNT_DEC,
        .counter_h_lim   = 32767,
        .counter_l_lim   = -32768,
        .unit            = _pcntUnit,
        .channel         = PCNT_CHANNEL_0
    };
    pcnt_unit_config(&pcnt_ch0);
    
    pcnt_config_t pcnt_ch1 = {
        .pulse_gpio_num  = this->_encoderConfig.pinB,
        .ctrl_gpio_num   = this->_encoderConfig.pinA,
        .lctrl_mode      = PCNT_MODE_KEEP,
        .hctrl_mode      = PCNT_MODE_REVERSE,
        .pos_mode        = PCNT_COUNT_INC,
        .neg_mode        = PCNT_COUNT_DEC,
        .counter_h_lim   = 32767,
        .counter_l_lim   = -32768,
        .unit            = _pcntUnit,
        .channel         = PCNT_CHANNEL_1
    };
    pcnt_unit_config(&pcnt_ch1);
    
    pcnt_event_enable(_pcntUnit, PCNT_EVT_H_LIM);
    pcnt_event_enable(_pcntUnit, PCNT_EVT_L_LIM);

    static bool isr_service_installed = false;
    if (!isr_service_installed) {
        pcnt_isr_service_install(0);
        isr_service_installed = true;
    }

    pcnt_isr_handler_add(_pcntUnit, ESP32Encoder::isr_handler, (void*)this);

    pcnt_counter_pause(_pcntUnit);
    pcnt_counter_clear(_pcntUnit);
    pcnt_counter_resume(_pcntUnit);
    
    return true;
}

void ESP32Encoder::Stop() {
    pcnt_counter_pause(_pcntUnit);
    pcnt_isr_handler_remove(_pcntUnit);
    pcnt_event_disable(_pcntUnit, PCNT_EVT_H_LIM);
    pcnt_event_disable(_pcntUnit, PCNT_EVT_L_LIM);
    pcnt_counter_clear(_pcntUnit);
}

void ESP32Encoder::isr_handler(void *arg){
    ESP32Encoder* obj = static_cast<ESP32Encoder*>(arg) ;
    uint32_t status = 0;
    
    // Get the interrupt status for this unit
    pcnt_get_event_status(obj->_pcntUnit, &status);

    if (status & PCNT_EVT_H_LIM) {
        obj->_overflowCounter++;
    }
    if (status & PCNT_EVT_L_LIM) {
        obj->_overflowCounter--;
    }    
}

int32_t ESP32Encoder::ReadSensor() { 
    int16_t hw_counter;
    pcnt_get_counter_value(_pcntUnit, &hw_counter);
    return (_overflowCounter*32768 + hw_counter);
}

}