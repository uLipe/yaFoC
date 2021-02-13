#pragma once 
#include "yaFoC.h"

namespace yafoc {
namespace platform_arduino {

using namespace rotor_sensor;

class WiringRotorAnalogSensor : public RotorSensorInterface<WiringRotorAnalogSensor>{
    float current_counter;
    float cpr;
    float prevous_read;
    float limit_low;
    float limit_high;
    int pin_number;

public:
    WiringRotorAnalogSensor(int sensor_pin, 
                            float minimum_count, 
                            float maximum_count);
    ~WiringRotorAnalogSensor();
    void SetCountToZero();
    float GetCountsPerRevolution(); 
    float ReadCounter();
};

}
}