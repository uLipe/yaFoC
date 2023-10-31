#pragma once

namespace yafoc {
namespace inverter_driver{

class InverterDriver {
public:
    InverterDriver(float m_dc_link_volts,
                int pin_u,
                int pin_v,
                int pin_w,
                int pin_enable_u,
                int pin_enable_v,
                int pin_enable_w
    );

    virtual ~InverterDriver();
    virtual int Disable() = 0;
    virtual int Enable() = 0;
    virtual int SetVoltages(float dc_u, float dc_v, float dc_w) = 0;

protected:
    int m_pins[3];
    int m_enable_pins[3];
    float m_dc_link_volts;
};

}
}
