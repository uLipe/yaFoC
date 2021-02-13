#pragma once 
#include <Arduino.h>
#include "yaFoC.h"

namespace yafoc {
namespace platform_arduino {

using namespace inverter_driver;

class Wiring3PhaseInverter : public InverterInterface<Wiring3PhaseInverter>{
    int pwm_u;
    int pwm_v;
    int pwm_w;
    float dc_link_voltage;
    float voltage_to_duty_scale;
public:
    Wiring3PhaseInverter(int pin_u, 
                        int pin_v, 
                        int pin_w, 
                        float link_voltage) : 
                        pwm_u(pin_u),
                        pwm_v(pin_v),
                        pwm_w(pin_w),
                        dc_link_voltage(link_voltage) {
    
        voltage_to_duty_scale = 255.0f / dc_link_voltage;  
        analogWrite(pwm_u,0);
        analogWrite(pwm_v,0);
        analogWrite(pwm_w,0);
    }

    ~Wiring3PhaseInverter() {
        analogWrite(pwm_u,0);
        analogWrite(pwm_v,0);
        analogWrite(pwm_w,0);
    }

    float GetVoltageSupply() {
        return dc_link_voltage;
    }

    void SetInverterVoltages(float u, float v, float w) {
        analogWrite(pwm_u,(int)(u * voltage_to_duty_scale));
        analogWrite(pwm_v,(int)(v * voltage_to_duty_scale));
        analogWrite(pwm_w,(int)(w * voltage_to_duty_scale));        
    }
};

}
}

