#pragma once 
#include "yaFoC.h"

namespace yafoc {
namespace platform_esp32 {

using namespace inverter_driver;

class ESP32_Ledc_Inverter : public InverterInterface<ESP32_Ledc_Inverter>{
    float dc_link_voltage;
    int pwm_uh;
    int pwm_vh;
    int pwm_wh;
    int pin_enable;
    float voltage_to_duty_scale;
public:
    ESP32_Ledc_Inverter(float link_voltage, int pin_u, int pin_v, int pin_w, int enable);
    ~ESP32_Ledc_Inverter();
    float GetVoltageSupply();
    void SetInverterVoltages(float u, float v, float w);
};

}
}