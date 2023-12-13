#include <functional>
#include <cmath>
#include <algorithm>
#include <memory>
#include "yaFoC.h"

namespace yafoc {

yaFoCMotorController::yaFoCMotorController(MotorHardware& motor, float pole_pairs, float sample_time_seconds) :
m_motor_hardware(motor), m_motor_pole_pairs(pole_pairs) {

    m_dt = sample_time_seconds;
    m_motor_hardware.FetchAllSensors();

    //First of all align the rotor:
    float rotor_angle;
    m_motor_hardware.GetRotorAngle(rotor_angle);
    float prev_angle = rotor_angle;

    m_motor_hardware.SetDutyCycles(0.0f, 0.0f, 0.0f);
    m_motor_hardware.SetDutyCycles(0.2f, 0.0f, 0.0f);

    do {
        m_motor_hardware.FetchAllSensors();
        m_motor_hardware.GetRotorAngle(rotor_angle);
        if(fabs(rotor_angle - prev_angle) < 0.5f)
            break;
        prev_angle = rotor_angle;
    } while (1);

    m_motor_hardware.SetDutyCycles(0.0f, 0.0f, 0.0f);
    do {
        m_motor_hardware.FetchAllSensors();
        m_motor_hardware.GetRotorAngle(rotor_angle);
        if(fabs(rotor_angle - prev_angle) < 0.5f)
            break;
        prev_angle = rotor_angle;
    } while (1);

    m_motor_hardware.GetRotorSpeedDps(m_shaft_speed);
    m_shaft_speed *= m_dt;
}

yaFoCMotorController::~yaFoCMotorController() {

}

bool yaFoCMotorController::SetPositionPid(PidController& pid) {
    m_position_pid = std::addressof(pid);
}

bool yaFoCMotorController::SetSpeedPid(PidController& pid) {
    m_speed_pid = std::addressof(pid);
}

void yaFoCMotorController::SetTargetSpeed(const float deg_per_second){
    m_target_speed = deg_per_second;
}

void yaFoCMotorController::SetTargetPosition(const float degrees) {
    m_target_position = degrees;
}

void yaFoCMotorController::Run() {
    if(PollNotification()) {
        m_motor_hardware.FetchAllSensors();
        float e_rotor_angle;
        float bus_voltage;
        float pwm_scale;
        float rotor_speed;
        m_motor_hardware.GetRotorSpeedDps(rotor_speed);
        rotor_speed *= m_dt;
        float mech_acc_angle;
        m_motor_hardware.GetAccumulatedAngle(mech_acc_angle);
        float mech_angle;
        m_motor_hardware.GetRotorAngle(mech_angle);
        GetElectricalAngle(mech_angle, e_rotor_angle);
        m_motor_hardware.GetDcBusVoltage(bus_voltage);
        m_motor_hardware.GetSpeedToPWMScale(pwm_scale);

        if(m_position_pid != nullptr) {
            m_position_pid_ratio--;
            if(!m_position_pid_ratio) {
                m_position_pid_ratio = 10;

                m_position_pid->Update(m_target_position,
                                    mech_acc_angle,
                                    m_u_position);
            }
        }

        if(m_speed_pid != nullptr) {
            m_speed_pid->Update(m_target_speed + m_u_position,
                            rotor_speed,
                            m_u_speed);
        } else {
            m_u_speed = m_target_speed * pwm_scale;
        }

        SetPhaseVoltage(m_u_speed * pwm_scale, 0.0f, e_rotor_angle);
        m_shaft_speed = rotor_speed;
        ConsumeNotification();
    }
}

void yaFoCMotorController::SetPhaseVoltage(float uq, float ud, float phase) {
    constexpr auto sqr3 =  1.7320508075688773f;
    constexpr auto sqrt3_by_two = sqr3 / 2.0f;

    float sine, cosine;
    float a,b,c;
    float alpha, beta;

    //Inverse Park transform to project Uq/d into Ualpha/beta
    //Rotating frame
    SinCos(phase, sine, cosine);
    alpha = ud * cosine - uq * sine;
    beta = ud * sine + uq * cosine;

    //Inverse Clark transform to project Ualpha/beta rotating
    //frame into Va/b/c three-phase rotating frame:
    a = alpha + 0.5f;
	b = (-0.5f * alpha + sqrt3_by_two * beta) + 0.5f;
	c = (-0.5f * alpha - sqrt3_by_two * beta) + 0.5f;

    if(a > 1.0f) {
        a = 1.0f;
    } else if(a < 0.0f) {
        a = 0.0f;
    }

    if(b > 1.0f) {
        b = 1.0f;
    } else if(a < 0.0f) {
        b = 0.0f;
    }

    if(c > 1.0f) {
        c = 1.0f;
    } else if(a < 0.0f) {
        c = 0.0f;
    }

    //Set the duty-cycles:
    m_motor_hardware.SetDutyCycles(a, b, c);
}

void yaFoCMotorController::GetElectricalAngle(float mechanical_angle, float& elec_angle) {
    elec_angle = mechanical_angle * m_motor_pole_pairs;
}

}