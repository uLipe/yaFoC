#pragma once

namespace yafoc {
namespace current_sensor{

class CurrentSensor {

public:
    CurrentSensor(float adc_to_amps_gain);
    virtual ~CurrentSensor();

    virtual int CalibrateSensor();
    virtual int FetchCurrents() = 0;
    virtual int GetPhaseCurrents(const float (&phase_currents) [3]);

protected:
    float m_adc_to_amps_gain;
    float m_phase_currents[3];
    float m_offsets[3];

};

}
}
