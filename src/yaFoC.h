#pragma once

#include <atomic>
#include "PidController.h"
#include "MotorHardware.h"
#include "MotorHardwareStub.h"
#include "Platform.h"
#include "sin_cos/sin_cos.h"

namespace yafoc {

class yaFoCMotorController : public Platform {
public:
    yaFoCMotorController(MotorHardware& motor, float pole_pairs, float sample_time_seconds);
    virtual ~yaFoCMotorController();

    bool SetPositionPid(PidController& pid);
    bool SetSpeedPid(PidController& pid);

    void SetTargetSpeed(const float deg_per_second);
    void SetTargetPosition(const float degrees);
    void Run();

private:
    void SetPhaseVoltage(float uq, float ud, float phase);
    void GetElectricalAngle(float mechanical_angle, float& elec_angle);

    MotorHardware& m_motor_hardware;
    float m_motor_pole_pairs{0.0f};
    float m_shaft_speed{0.0f};

    PidController *m_speed_pid;
    PidController *m_position_pid;

    int m_position_pid_ratio{10};
    std::atomic<float> m_target_speed{0.0f};
    std::atomic<float> m_target_position{0.0f};
    float m_u_position{0.0f};
    float m_u_speed{0.0f};
};
}