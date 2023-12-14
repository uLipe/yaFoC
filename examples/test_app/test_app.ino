#include <Arduino.h>
#include <yaFoC.h>

yafoc::MotorStub motor;
yafoc::yaFoCMotorController motor_controller(motor, 12.0f, 1.0f);
yafoc::PidController speed_pid(-1000, 1000);

void setup() {
    motor_controller.SetSpeedPid(speed_pid);
    motor_controller.SetTargetSpeed(360.0f);
}

void loop () {
    motor_controller.Trigger();
    motor_controller.Run();
}
