#pragma once 
#include "yaFoC.h"

namespace yafoc {
namespace platform_arduino {

using namespace inverter_driver;

class Wiring3PhaseInverter : public InverterInterface<Wiring3PhaseInverter>{
    float dc_link_voltage;
    int pwm_u;
    int pwm_v;
    int pwm_w;
    float voltage_to_duty_scale;
public:
    Wiring3PhaseInverter(int pin_u, int pin_v, int pin_w, float link_voltage);
    ~Wiring3PhaseInverter();
    float GetVoltageSupply();
    void SetInverterVoltages(float u, float v, float w);
};

}
}