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

    class ControllerFOC {

    public:
        ControllerFOC();
        ~ControllerFOC();
        RotorAlignStatus AlignRotor();
        RotorAlignStatus SetTargetCurrent();
        void RunControllerFOC();
    };

}