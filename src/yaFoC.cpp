#include "include/yaFoC/dq_transform.h"
#include "include/yaFoC/electric_angle.h"
#include "include/yaFoC/pid_controller.h"
#include "include/yaFoC/trigonometry_utils.h"

#include "yaFoC.h"

namespace yafoc {

    ControllerFOC::ControllerFOC() {

    }

    ControllerFOC::~ControllerFOC() {

    }

    RotorAlignStatus ControllerFOC::AlignRotor() {
        return RotorAlignStatus::kAligned;
    }

    RotorAlignStatus ControllerFOC::SetTargetCurrent() {
        return RotorAlignStatus::kAligned;
    }

    void ControllerFOC::RunControllerFOC() {
        
    }

}