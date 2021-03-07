#pragma once 
#include "yaFoC.h"

namespace yafoc {
namespace platform_open_loop {

using namespace current_sensor;

class OpenCurrentSensor : public CurrentSensorInterface<OpenCurrentSensor>{

public:
    OpenCurrentSensor() {

    }

    ~OpenCurrentSensor() {

    }

    PhaseCurrentsAmperes GetPhaseCurrents() {
           auto phase_current = current_sensor::PhaseCurrentsAmperes(
                                (float)0.0f,
                                (float)0.0f,
                                (float)0.0f); 
    }
};

}
}