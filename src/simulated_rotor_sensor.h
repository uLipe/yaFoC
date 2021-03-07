#pragma once 
#include "yaFoC.h"
#include <Arduino.h>

namespace yafoc {
namespace platform_simulator {

using namespace rotor_sensor;

class SimulatedRotorSensor : public RotorSensorInterface<SimulatedRotorSensor>{
    float count;
    float delta;

public:
    SimulatedRotorSensor(float simulated_increase) : delta(simulated_increase + 1.0f) {
        count = 0.0f;
    }

    ~SimulatedRotorSensor() {

    }

    void SetCountToZero() {
        count = 0.0f;
    }

    float GetCountsPerRevolution() {
        return 500.0f;
    } 

    float ReadCounter() {
        float result = count;
        count += delta;

        if(count > 1000000.0f) {
            count = 0.0f + delta;
        } else if (count < 0.0f) {
            count = 100000.0f - delta;;
        }

        delay(5);

        return count;
    }

    void SensorBlockingDelayMs(unsigned ms) {
        delay(ms);
    }
};

}
}