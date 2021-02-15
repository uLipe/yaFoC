#pragma once 

#include "include/yaFoC/dq_transform.h"
#include "include/yaFoC/electric_angle.h"
#include "include/yaFoC/pid_controller.h"
#include "include/yaFoC/trigonometry_utils.h"
#include "include/yaFoC/modulator.h"

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

struct FoCTelemetry {
    float iq;
    float id;
    float shaft_ticks;
    float rotor_position_mechanical;
    float rotor_position_electrical;
    float phase_voltages[3];
    float phase_currents[3];
    float supply_voltage;
    float dt;

    FoCTelemetry() {};
};

template <class Current, class Rotor, class Driver>
class ControllerFOC {
    DCurrent i_d;
    QCurrent i_q;
    Radians  shaft_angle;
    TimeStamp last_t;
    SupplyVoltage supply_voltage;
    
    float motor_pole_pairs;
    current_sensor::CurrentSensorInterface<Current>& i_sensor;
    rotor_sensor::RotorSensorInterface<Rotor>& shaft_sensor;
    inverter_driver::InverterInterface<Driver>& pwm_driver;
    
    float dt;
    RotorAlignStatus status;
    
public:
    ControllerFOC(unsigned pole_pairs, 
                current_sensor::CurrentSensorInterface<Current>& cs,
                rotor_sensor::RotorSensorInterface<Rotor>& shaft,
                inverter_driver::InverterInterface<Driver>& inverter) : 
                i_d(0.0f),
                i_q(0.0f),
                shaft_angle(0.0f),
                last_t(0.0f),
                supply_voltage(0.0f),
                motor_pole_pairs((float)pole_pairs),
                i_sensor(cs),
                shaft_sensor(shaft),
                pwm_driver(inverter) {

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

    void GetTelemetry(FoCTelemetry& telemetry) {
        
    } 

    void RunControllerFOC(TimeStamp& now){
        (void)now.ToRaw();
    }
};
}