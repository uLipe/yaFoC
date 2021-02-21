#pragma once 

#include "include/yaFoC/dq_transform.h"
#include "include/yaFoC/electric_angle.h"
#include "include/yaFoC/pid_controller.h"
#include "include/yaFoC/trigonometry_utils.h"
#include "include/yaFoC/modulator.h"

#include "platform_current_sensor.h"
#include "platform_inverter_driver.h"
#include "platform_rotor_sensor.h"

#include <math.h>

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
    SupplyVoltage biased_supply_voltage;
  
    float motor_pole_pairs;
    current_sensor::CurrentSensorInterface<Current>& i_sensor;
    rotor_sensor::RotorSensorInterface<Rotor>& shaft_sensor;
    inverter_driver::InverterInterface<Driver>& pwm_driver;
    
    float dt;
    float shaft_to_radians;
    RotorAlignStatus status;

    inline float ShaftTicksToRadians() {
        using namespace conversions;
        shaft_angle.raw = shaft_to_radians * shaft_sensor.ReadCounter();
        return NormalizeElectricalAngle(FromMechanicalToElectricAngle(shaft_angle.raw, motor_pole_pairs));    
    }
    
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
                biased_supply_voltage(0.0f),
                motor_pole_pairs((float)pole_pairs),
                i_sensor(cs),
                shaft_sensor(shaft),
                pwm_driver(inverter) {

        status = RotorAlignStatus::kNotAligned;
        supply_voltage.raw = pwm_driver.GetVoltageSupply();
        biased_supply_voltage.raw = supply_voltage.raw * 0.5f;
        shaft_to_radians = ((2.0f * M_PI) /shaft_sensor.GetCountsPerRevolution());
        pwm_driver.SetInverterVoltages(0.0f, 0.0f, 0.0f);
    }

    ~ControllerFOC() {
        status = RotorAlignStatus::kNotAligned;
        pwm_driver.SetInverterVoltages(0.0f, 0.0f, 0.0f);
    }

    RotorAlignStatus InitializeAndAlignRotor(){
        using namespace modulator;
        using namespace conversions;

        float  vqd[2] = {0.0f, 0.0f};
        float  vabc[3];
        float  theta = 0.0f;

        ModulateDqVoltages(biased_supply_voltage.ToRaw(), 
                        theta, 
                        vqd[0], 
                        vqd[1], 
                        &vabc[0], 
                        &vabc[1], 
                        &vabc[2]);
        pwm_driver.SetInverterVoltages(vabc[0], vabc[1], vabc[2]);
        shaft_sensor.SensorBlockingDelayMs(1000);

        vqd[0] = supply_voltage.ToRaw() * 0.1f;
        vqd[1] = 0.0f;

        for (float i = 0.0f; i < 5.0f; i += 1.0f) {
            theta = NormalizeElectricalAngle(FromMechanicalToElectricAngle((ALIGN_ANGLE_CONSTANT * i) / 6.0f, motor_pole_pairs));
            ModulateDqVoltages(biased_supply_voltage.ToRaw(), 
                            theta, 
                            vqd[0], 
                            vqd[1], 
                            &vabc[0], 
                            &vabc[1], 
                            &vabc[2]);
            pwm_driver.SetInverterVoltages(vabc[0], vabc[1], vabc[2]);
            shaft_sensor.SensorBlockingDelayMs(250);
        }

        for (float i = 5.0f; i != 0.0f; i -= 1.0f) {
            theta = NormalizeElectricalAngle(FromMechanicalToElectricAngle((ALIGN_ANGLE_CONSTANT * i) / 6.0f, motor_pole_pairs));
            ModulateDqVoltages(biased_supply_voltage.ToRaw(), 
                            theta, 
                            vqd[0], 
                            vqd[1], 
                            &vabc[0], 
                            &vabc[1], 
                            &vabc[2]);
            pwm_driver.SetInverterVoltages(vabc[0], vabc[1], vabc[2]);
            shaft_sensor.SensorBlockingDelayMs(250);
        }

        shaft_sensor.SensorBlockingDelayMs(1000);
        shaft_sensor.SetCountToZero();
        shaft_sensor.SensorBlockingDelayMs(1000);

        vqd[0] = 0.0f;
        vqd[1] = 0.0f;

        ModulateDqVoltages(biased_supply_voltage.ToRaw(), 
                        theta, 
                        vqd[0], 
                        vqd[1], 
                        &vabc[0], 
                        &vabc[1], 
                        &vabc[2]);
        pwm_driver.SetInverterVoltages(vabc[0], vabc[1], vabc[2]);
        shaft_sensor.SensorBlockingDelayMs(1000);
        status = RotorAlignStatus::kAligned; 

        return status;
    }

    RotorAlignStatus SetTargetCurrent(DCurrent& id, QCurrent& iq){
        if(status != RotorAlignStatus::kAligned) {
            return status;
        }

        i_q.raw = iq.ToRaw();
        i_d.raw = id.ToRaw();
        return status;
    }

    void GetTelemetry(FoCTelemetry& telemetry) {
    } 

    void RunControllerFOC(TimeStamp& now){
        (void)now.ToRaw();
    }
};
}