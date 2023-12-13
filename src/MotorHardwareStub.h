#pragma once

#include "MotorHardware.h"

namespace yafoc {

class MotorStub : public MotorHardware {

public:
    MotorStub() {};
    ~MotorStub() {};

    void GetRotorAngle(float& angle) override {
        angle = 0.0f;
    }

    void GetAccumulatedAngle(float& acc_angle) override {
        acc_angle = 0.0f;
    }

    void GetRotorSpeedDps(float& speed) override {
        speed = 0.0f;
    }

    void SetDutyCycles(float a, float b, float c) override {
        (void)a;
        (void)b;
        (void)c;
    }

    void CommandPowerStage(bool enable) override {
        (void)enable;
    }

    void FetchAllSensors() override {

    }

    void GetDcBusVoltage(float& bus_voltage) override {
        bus_voltage = 12.0f;
    }

    void GetSpeedToPWMScale(float &scale) override {
        scale = 1.0f;
    }
};

}


