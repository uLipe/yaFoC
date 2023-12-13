#pragma once

namespace yafoc {

class MotorHardware {
public:
    virtual void FetchAllSensors() = 0;
    virtual void GetRotorAngle(float& angle) = 0;
    virtual void GetAccumulatedAngle(float& acc_angle) = 0;
    virtual void GetRotorSpeedDps(float& speed) = 0;
    virtual void SetDutyCycles(float a, float b, float c) = 0;
    virtual void CommandPowerStage(bool enable) = 0;
    virtual void GetDcBusVoltage(float &bus_voltage) = 0;
    virtual void GetSpeedToPWMScale(float &scale) = 0;
};

}