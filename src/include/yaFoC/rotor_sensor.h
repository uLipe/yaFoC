#pragma once

namespace yafoc {
namespace rotor_sensor{

class RotorSensor {

public:
    RotorSensor(int pulses_per_revolution);
    virtual ~RotorSensor();

    virtual int FetchPosition() = 0;
    virtual float GetPosition() = 0;

protected:
    float m_pulses_per_revolution;
    float m_pulses_to_degrees_gain;
};

}
}
