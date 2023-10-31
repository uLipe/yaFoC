#include "yaFoC.h"

yaFoCMotorController::ControllerFOC(float pole_pairs, int control_loop_time_us, RotorSensor& r, InverterDriver& i, TimerDriver& t) :
    m_motor_pole_pairs{pole_pairs},
    m_control_loop_time_us{control_loop_time_us},
    m_rotor_sensor{r},
    m_inverter{i},
    m_timer{t}
{
    m_inverter->Disable();
    m_inverter->SetVoltages(0,0,0);

    auto motor_callback = [this] (float dt) {
        float cmd_speed{0};
        float cmd_iq{0}

        /* Read sensors before doing anything*/
        m_shaft_angle = rotor_position.GetPosition();

        auto ea_normalized = conversions::NormalizeElectricalAngle(
                                conversions::FromMechanicalToElectricAngle(
                                    m_shaft_angle
                                ));

        //Observe the new speed:
        m_speed_observer.m_input_position = m_shaft_angle;
        ObserveSpeedFromPosition(m_speed_observer);
        m_shaft_speed = m_speed_observer.m_output_speed;

        if(current_sensor != nullptr) {
            m_current_sensor.GetPhaseCurrents(m_iabc);
        }

        if(m_foc_chain_controllers[static_cast<int>(kPositionPid)] != nullptr) {
            PidController* p = m_foc_chain_controllers[static_cast<int>(kPositionPid)];
            cmd_speed = p->Update(m_target_position, m_shaft_angle);
        }

        if(m_foc_chain_controllers[static_cast<int>(kSpeedPid)] != nullptr) {
            PidController* p = m_foc_chain_controllers[static_cast<int>(kSpeedPid)];
            cmd_iq = p->Update(m_target_speed + cmd_speed, m_shaft_speed);
        }

        if((m_foc_chain_controllers[static_cast<int>(kIdPid)] != nullptr) &&
           (m_foc_chain_controllers[static_cast<int>(kIqPid)] != nullptr)) {
            //For current control both IQ and IQ controllers must exist:
            PidController* iqC = m_foc_chain_controllers[static_cast<int>(kIdPid)];
            PidController* idC = m_foc_chain_controllers[static_cast<int>(kIqPid)];

            //Project measured currents into dq_frame:
            modulator::GetDqCurrents(ea_normalized,
                                    m_iabc[0],
                                    m_iabc[1],
                                    m_iabc[2],
                                    m_iqd_measured[1],
                                    m_iqd_measured[0]);

            m_vqd[0] = iqC->Update(cmd_iq, m_iqd_measured[0]);
            m_vqd[1] = iqC->Update(0.0f, m_iqd_measured[0]);
        } else {
            //No current control required:
            m_vqd[0] = cmd_iq;
            m_vqd[1] = 0.0f
        }

        modulator::ModulateDqVoltages(0.5f,
                                    ea_normalized,
                                    m_vqd[1],
                                    m_vqd[0],
                                    m_vabc[0],
                                    m_vabc[1],
                                    m_vabc[2]);

        m_inverter.SetVoltages(m_vabc[0], m_vabc[1], m_vabc[2]);
    };

    m_timer->RegisterCallback(motor_callback, NULL);
}

yaFoCMotorController::RotorAlignStatus AlignRotor() {

    m_timer->Start(m_control_loop_time_us);
    return kAligned;
}

yaFoCMotorController::int LinkCurrentSensor(CurrentSensor& s) {
    m_current_sensor = std::addressof(s);
    return 0;
}

yaFoCMotorController::int LinkPidController(const PidController& pid, PidControlIndex idx) {
    m_foc_chain_controllers[static_cast<int>(idx)] = std::addressof(pid);
    return 0;
}

yaFoCMotorController::void SetTargetSpeed(const float rpm) {
    //RPM -> Degree per second.
    m_target_speed = rpm * 5.9999995176f;
}

yaFoCMotorController::void SetTargetPosition(const float degrees) {
    m_target_position = degrees;
}

yaFoCMotorController::void GetMotorTelemetry(FocTelemetry& telemetry) {
    telemetry.vabc[0] = m_vabc[0];
    telemetry.vabc[1] = m_vabc[1];
    telemetry.vabc[2] = m_vabc[2];
    telemetry.rotor_position = m_shaft_angle;
    telemetry.rotor_speed = m_shaft_speed;
    telemetry.rotor_position_ref = m_target_position;
    telemetry.rotor_speed_ref = m_target_speed;
    telemetry.vq = m_vqd[0];
    telemetry.vd = m_vqd[1];
    telemetry.iq = m_iqd_measured[0];
    telemetry.id = m_iqd_measured[1];
}