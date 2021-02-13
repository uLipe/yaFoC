#pragma once 
#include <Arduino.h>
#include "yaFoC.h"

namespace yafoc {
namespace platform_arduino {

using namespace current_sensor;

class WiringCurrentSensor : public CurrentSensorInterface<WiringCurrentSensor>{
    float adc_to_amperes_gain;
    int adc_u;
    int adc_v;
    int adc_w;

public:
    WiringCurrentSensor(int pin_u, 
                                int pin_v, 
                                int pin_w, 
                                float gain) : 
                                adc_to_amperes_gain(gain),
                                adc_u(pin_u),
                                adc_v(pin_v),
                                adc_w(pin_w) {}

    ~WiringCurrentSensor() {}
    
    PhaseCurrentsAmperes GetPhaseCurrents() {
        auto phase_current = current_sensor::PhaseCurrentsAmperes(
                            (float)analogRead(adc_u) * adc_to_amperes_gain,
                            (float)analogRead(adc_v) * adc_to_amperes_gain,
                            (float)analogRead(adc_w) * adc_to_amperes_gain);


        return phase_current;
    }
};

}
}

