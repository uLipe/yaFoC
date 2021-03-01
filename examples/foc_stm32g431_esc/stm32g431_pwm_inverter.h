#pragma once 
#include "yaFoC.h"

namespace yafoc {
namespace platform_stm32 {

using namespace inverter_driver;

class Stm32g431_Inverter : public InverterInterface<Stm32g431_Inverter>{
    int pwm_uh;
    int pwm_ul;
    int pwm_vh;
    int pwm_vl;
    int pwm_wh;
    int pwm_wl;
    float dc_link_voltage;
    float voltage_to_duty_scale;
public:
    Stm32g431_Inverter(float link_voltage);
    ~Stm32g431_Inverter();
    float GetVoltageSupply();
    void SetInverterVoltages(float u, float v, float w);
};

}
}