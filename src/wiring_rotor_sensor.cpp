#include <Arduino.h>
#include "wiring_rotor_sensor.h"
extern "C" {
    #include <math.h>
}


namespace yafoc {
namespace platform_arduino {

WiringRotorAnalogSensor::WiringRotorAnalogSensor(int sensor_pin, 
                                            float minimum_count, 
                                            float maximum_count) : 
                                            pin_number(sensor_pin){

    current_counter = 0.0f;
    prevous_read = analogRead(pin_number);
    cpr = maximum_count - minimum_count;
    limit_high = 100.0f * cpr;
    limit_low  = -100.0f * cpr;
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
    
        if(current_counter > limit_high) {
            current_counter -= limit_high;
        }else if (current_counter < limit_low) {
            current_counter -= limit_low;
        }
    }

    prevous_read = raw;

    return current_counter + raw;
}

void WiringRotorAnalogSensor::SensorBlockingDelayMs(unsigned ms) {
    delay(ms);
}

}
}