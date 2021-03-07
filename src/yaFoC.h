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
    controller::PidController iq_controller;
    controller::PidController id_controller;

    float dt;
    float shaft_to_radians;
    float vqd[2];
    float vabc[3];
    float iqd_measured[2];
    bool open_loop_current_mode;
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
                pwm_driver(inverter),
                iq_controller(),
                id_controller() {

        status = RotorAlignStatus::kNotAligned;
        supply_voltage.raw = pwm_driver.GetVoltageSupply();
        biased_supply_voltage.raw = supply_voltage.raw * 0.5f;
        shaft_to_radians = ((2.0f * M_PI) /shaft_sensor.GetCountsPerRevolution());

        iq_controller.kp = 10.0f;
        iq_controller.kp = 1.0f;
        iq_controller.kd = 0.0f;

        id_controller.kp = 10.0f;
        id_controller.kp = 1.0f;
        id_controller.kd = 0.0f;
        
        open_loop_current_mode = true;

        pwm_driver.SetInverterVoltages(0.0f, 0.0f, 0.0f);
    }

    ~ControllerFOC() {
        status = RotorAlignStatus::kNotAligned;
        pwm_driver.SetInverterVoltages(0.0f, 0.0f, 0.0f);
    }

    RotorAlignStatus InitializeAndAlignRotor(){
        using namespace modulator;
        using namespace conversions;
        float  theta = 0.0f;
        vqd[0] = 0.0f;
        vqd[1] = supply_voltage.ToRaw() * 0.1f;

        for (float i = 0.0f; i < 500.0f; i += 1.0f) {
            theta = NormalizeElectricalAngle(FromMechanicalToElectricAngle((ALIGN_ANGLE_CONSTANT * i) / 500.0f, motor_pole_pairs));
            ModulateDqVoltages(biased_supply_voltage.ToRaw(), 
                            theta, 
                            vqd[0], 
                            vqd[1], 
                            &vabc[0], 
                            &vabc[1], 
                            &vabc[2]);
            pwm_driver.SetInverterVoltages(vabc[0], vabc[1], vabc[2]);
            shaft_sensor.SensorBlockingDelayMs(5);
        }

        for (float i = 500.0f; i != 0.0f; i -= 1.0f) {
            theta = NormalizeElectricalAngle(FromMechanicalToElectricAngle((ALIGN_ANGLE_CONSTANT * i) / 500.0f, motor_pole_pairs));
            ModulateDqVoltages(biased_supply_voltage.ToRaw(), 
                            theta, 
                            vqd[0], 
                            vqd[1], 
                            &vabc[0], 
                            &vabc[1], 
                            &vabc[2]);
            pwm_driver.SetInverterVoltages(vabc[0], vabc[1], vabc[2]);
            shaft_sensor.SensorBlockingDelayMs(5);
        }

        shaft_sensor.SensorBlockingDelayMs(250);
        pwm_driver.SetInverterVoltages(0.0f, 0.0f, 0.0f);
        shaft_sensor.SensorBlockingDelayMs(250);
        shaft_sensor.SetCountToZero();
        status = RotorAlignStatus::kAligned; 

        return status;
    }

    void SetControllerToClosedLoop() {
        open_loop_current_mode = false;
    }

    void SetControllerToOpenLoop() {
        open_loop_current_mode = true;
    }

    RotorAlignStatus SetTargetCurrent(DCurrent& id, QCurrent& iq){
        if(status != RotorAlignStatus::kAligned) {
            return status;
        }

        i_q.raw = iq.ToRaw();
        i_d.raw = id.ToRaw();
        return status;
    }

    RotorAlignStatus RunControllerFOC(TimeStamp& now){
        using namespace modulator;
        using namespace conversions;

        if(status != RotorAlignStatus::kAligned) {
            return status;
        }

        float dt = (now.ToRaw() - last_t.ToRaw());
        last_t.raw = now.ToRaw();

        if(dt == 0.0f) {
            return RotorAlignStatus::kInvalid;
        }

        float theta = ShaftTicksToRadians();
        auto iabc = i_sensor.GetPhaseCurrents();

        GetDqCurrents(theta, 
                    iabc.i_u, 
                    iabc.i_v,
                    iabc.i_w,
                    &iqd_measured[0],
                    &iqd_measured[1]);
                    
        if(!open_loop_current_mode){
            auto setpoint_q = controller::Setpoint(i_q.ToRaw());
            auto measured_q = controller::Measured(iqd_measured[0]);
            auto setpoint_d = controller::Setpoint(i_d.ToRaw());
            auto measured_d = controller::Measured(iqd_measured[1]);

            vqd[0] = iq_controller.Update(setpoint_q, measured_q, dt);
            vqd[1] = id_controller.Update(setpoint_d, measured_d, dt);

            vqd[0] = controller::SymmetricSaturate(vqd[0], biased_supply_voltage.ToRaw());
            vqd[1] = controller::SymmetricSaturate(vqd[1], biased_supply_voltage.ToRaw());
        } else {
            vqd[0] = i_q.ToRaw();
            vqd[1] = i_d.ToRaw();
        }

        ModulateDqVoltages(biased_supply_voltage.ToRaw(),
                        theta,
                        vqd[1],
                        vqd[0],
                        &vabc[0],
                        &vabc[1],
                        &vabc[2]);

        pwm_driver.SetInverterVoltages(vabc[0],vabc[1],vabc[2]);

        return status;
    }
};
}