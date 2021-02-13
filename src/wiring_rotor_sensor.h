#pragma once 
#include <Arduino.h>
#include "yaFoC.h"

namespace yafoc {
namespace platform_arduino {

using namespace rotor_sensor;

class WiringRotorAnalogSensor : public RotorSensorInterface<WiringRotorAnalogSensor>{
    int pin_number;
    float limit_low;
    float limit_high;
    float current_counter;
    float cpr;
    float prevous_read;

public:
    WiringRotorAnalogSensor(int sensor_pin, 
                        float minimum_count, 
                        float maximum_count) : 
                        pin_number(sensor_pin),
                        limit_high(maximum_count),
                        limit_low(minimum_count) {

        current_counter = 0.0f;
        prevous_read = analogRead(pin_number);
        cpr = maximum_count - minimum_count;
    }    
    
    ~WiringRotorAnalogSensor() {

    }

    void SetCountToZero() {
        current_counter = 0.0f;
        prevous_read = analogRead(pin_number);
    }

    float GetCountsPerRevolution() {
        return cpr;
    }

    float ReadCounter() {
        float raw = analogRead(pin_number);

        /* consider the analog wrap-around reading values */
        float delta = (raw - prevous_read);
        
        if(fabs(delta) > cpr) {
            current_counter = (delta < 0.0f) ? current_counter + cpr :
                                            current_counter - cpr;
        }
        prevous_read = raw;

        return current_counter + raw;
    }
};

}
}