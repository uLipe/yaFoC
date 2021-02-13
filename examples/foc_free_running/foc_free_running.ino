#include <Arduino.h>
#include <yaFoC.h>
#include <wiring_current_sensor.h>
#include <wiring_rotor_sensor.h>
#include <wiring_inverter.h>

yafoc::platform_arduino::WiringCurrentSensor phase_sensors(8,9,10, 1.0f);
yafoc::platform_arduino::WiringRotorAnalogSensor rotor_sensor(7, 10, 1023);
yafoc::platform_arduino::Wiring3PhaseInverter inverter(1,2,3,12.0f);

using Current = yafoc::platform_arduino::WiringCurrentSensor;
using Rotor = yafoc::platform_arduino::WiringRotorAnalogSensor;
using Driver = yafoc::platform_arduino::Wiring3PhaseInverter;

yafoc::ControllerFOC<Current,Rotor,Driver> motor_controller(11, 
                                                phase_sensors, 
                                                rotor_sensor, 
                                                inverter);

void setup() {
    motor_controller.InitializeAndAlignRotor();
}

void loop() {
    auto now = yafoc::TimeStamp((float)micros());
    motor_controller.RunControllerFOC(now);
}