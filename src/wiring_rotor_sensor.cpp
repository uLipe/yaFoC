#include <Arduino.h>
#include "wiring_rotor_sensor.h"
#include <cmath>

namespace yafoc {
namespace platform_arduino {

WiringRotorAnalogSensor::WiringRotorAnalogSensor(int sensor_pin, 
                                            float minimum_count, 
                                            float maximum_count) : 
                                            pin_number(sensor_pin),
                                            limit_high(maximum_count),
                                            limit_low(minimum_count){

    current_counter = 0.0f;
    prevous_read = analogRead(pin_number);
    cpr = maximum_count - minimum_count;
}

WiringRotorAnalogSensor::~WiringRotorAnalogSensor() {
    //Something should go there to gracefully stop sensor?
}

void WiringRotorAnalogSensor::SetCountToZero() {
    current_counter = 0.0f;
    prevous_read = analogRead(pin_number);
}

float WiringRotorAnalogSensor::GetCountsPerRevolution() {
    return cpr;
}

float WiringRotorAnalogSensor::ReadCounter() {
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

}
}