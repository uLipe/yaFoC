#pragma once 
#include "../yaFoC.h"

namespace yafoc {
namespace platform_arduino {

using namespace current_sensor;

class WiringCurrentSensor : public CurrentSensorInterface<WiringCurrentSensor>{
    float adc_to_amperes_gain;
    int adc_u;
    int adc_v;
    int adc_w;

public:
    WiringCurrentSensor(int pin_u, int pin_v, int pin_w, float gain);
    ~WiringCurrentSensor();
    PhaseCurrentsAmperes GetPhaseCurrents();  
};

}
}