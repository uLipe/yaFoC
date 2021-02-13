#include <Arduino.h>
#include "wiring_current_sensor.h"

namespace yafoc {
namespace platform_arduino {


WiringCurrentSensor::WiringCurrentSensor(int pin_u, 
                                int pin_v, 
                                int pin_w, 
                                float gain) : 
                                adc_to_amperes_gain(gain),
                                adc_u(pin_u),
                                adc_v(pin_v),
                                adc_w(pin_w) {

}

WiringCurrentSensor::~WiringCurrentSensor() {

}

PhaseCurrentsAmperes WiringCurrentSensor::GetPhaseCurrents() {

    auto phase_current = current_sensor::PhaseCurrentsAmperes(
                                (float)analogRead(adc_u) * adc_to_amperes_gain,
                                (float)analogRead(adc_v) * adc_to_amperes_gain,
                                (float)analogRead(adc_w) * adc_to_amperes_gain);


    return phase_current;
}

}
}