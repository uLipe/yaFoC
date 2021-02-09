#pragma once 

#include "include/yaFoC/dq_transform.h"
#include "include/yaFoC/electric_angle.h"
#include "include/yaFoC/pid_controller.h"
#include "include/yaFoC/trigonometry_utils.h"

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

    struct SupplyVoltage {
        float raw;
        SupplyVoltage(float initial): raw(initial) {};
        float ToRaw() {return raw;};
    };

    struct TimeStamp {
        float raw;
        TimeStamp(float initial): raw(initial) {};
        float ToRaw() {return raw;};
    };

    template <class A, class B, class C>
    class ControllerFOC {
        DCurrent i_d;
        QCurrent i_q;
        Radians  shaft_angle;
        TimeStamp last_t;
        SupplyVoltage supply_voltage;
        current_sensor::PhaseCurrentsAmperes i_phases;
        RotorAlignStatus status;
        float dt;
        float motor_pole_pairs;

        current_sensor::CurrentSensorInterface<A>& i_sensor;
        rotor_sensor::RotorSensorInterface<B>& shaft_sensor;
        inverter_driver::InverterInterface<C>& pwm_driver;
        
    public:
        ControllerFOC(unsigned pole_pairs, 
                    current_sensor::CurrentSensorInterface<A>& cs,
                    rotor_sensor::RotorSensorInterface<B>& shaft,
                    inverter_driver::InverterInterface<C>& inverter) : 
                    motor_pole_pairs((float)pole_pairs),
                    i_sensor(cs),
                    shaft_sensor(shaft),
                    pwm_driver(inverter),
                    i_d(0.0f),
                    i_q(0.0f),
                    last_t(0.0f) {

            status = RotorAlignStatus::kNotAligned;
            supply_voltage.raw = pwm_driver.GetVoltageSupply();
            pwm_driver.SetInverterVoltages(0.0f, 0.0f, 0.0f);
        }

        ~ControllerFOC() {
            status = RotorAlignStatus::kNotAligned;
            pwm_driver.SetInverterVoltages(0.0f, 0.0f, 0.0f);
        }

        RotorAlignStatus InitializeAndAlignRotor(){
            return RotorAlignStatus::kAligned;
        }

        RotorAlignStatus SetTargetCurrent(DCurrent& id, QCurrent& iq){
            return RotorAlignStatus::kAligned;
        }

        void RunControllerFOC(TimeStamp& now){
            
        }
    };
}