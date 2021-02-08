#pragma once 

#include "platform_current_sensor.h"
#include "platform_inverter_driver.h"
#include "platform_rotor_sensor.h"

namespace yafoc {

    enum class RotorAlignStatus{
        kNotAligned,
        kAligned,
        kInvalid
    };

    struct Radians {
        float raw;
        Radians(float initial): raw(initial) {};
        float ToRaw() {return raw;};
    };

    struct QCurrent {
        float raw;
        QCurrent(float initial): raw(initial) {};
        float ToRaw() {return raw;};
    };

    struct DCurrent {
        float raw;
        DCurrent(float initial): raw(initial) {};
        float ToRaw() {return raw;};
    };

    class ControllerFOC {

    public:
        ControllerFOC();
        ~ControllerFOC();
        RotorAlignStatus AlignRotor();
        RotorAlignStatus SetTargetCurrent();
        void RunControllerFOC();
    };

}