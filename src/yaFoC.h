#pragma once

#include <functional>
#include <algorithm>
#include <vector>
#include <cerrno.h>

#include "include/yaFoC/dq_transform.h"
#include "include/yaFoC/electric_angle.h"
#include "include/yaFoC/pid_controller.h"
#include "include/yaFoC/trigonometry_utils.h"
#include "include/yaFoC/modulator.h"

#include "include/yaFoC/current_sensor.h"
#include "include/yaFoC/inverter_driver.h"
#include "include/yaFoC/rotor_sensor.h"
#include "include/yaFoC/timer_driver.h"

namespace yafoc {

using namespace rotor_sensor;
using namespace timer_driver;
using namespace inverter_driver;
using namespace current_sensor;
using namespace controller;
using namespace speed_observer;

enum class RotorAlignStatus{
    kNotAligned,
    kAligned,
    kInvalid
};

enum class PidControlIndex{
    kIqPid = 0,
    kIdPid,
    kSpeedPid,
    kPositionPid,
    kPidIndexMax,
};

struct FocTelemetry {
    float vabc[3];

    float rotor_position_ref;
    float rotor_speed_ref;
    float rotor_position;
    float rotor_speed;
    float vq;
    float vd;
    float iq;
    float id;
    float dt;
};

class yaFoCMotorController {
public:
    yaFoCMotorController(float pole_pairs, int control_loop_time_us, RotorSensor& r, InverterDriver& i, TimerDriver& t);
    ~yaFoCMotorController() = delete;

    RotorAlignStatus AlignRotor();
    int LinkCurrentSensor(CurrentSensor& s);
    int LinkPidController(const PidController& pid, PidControlIndex idx);
    void SetTargetSpeed(const float rpm);
    void SetTargetPosition(const float degrees);
    void GetMotorTelemetry(FocTelemetry& telemetry);

private:
    void RunMotorControl(void *this, float dt);

    float m_motor_pole_pairs{0.0f};
    float m_shaft_angle{0.0f};
    float m_shaft_angle_prev{0.0f};
    float m_shaft_speed{0.0f};

    int m_control_loop_time_us;

    std::atomic<float> m_target_speed{0.0f};
    std::atomic<float> m_target_position{0.0f};

    float m_iabc[3];
    float m_iqd_measured[2];
    float m_vqd[2];
    float m_vabc[3];

    std::vector<PidController *, static_cast<int>(kPidIndexMax)> m_foc_chain_controllers;

    RotorSensor& m_rotor_sensor;
    InverterDriver& m_inverter;
    TimerDriver& m_timer;
    CurrentSensor* m_current_sensor;
    ObserverState m_speed_observer;
};
}