#include <Arduino.h>
#include <yaFoC.h>
#include <wiring_current_sensor.h>
#include <wiring_rotor_sensor.h>
#include "stm32g431_pwm_inverter.h"

yafoc::platform_arduino::WiringCurrentSensor phase_sensors(OP1_OUT,
                                                        OP2_OUT,
                                                        OP3_OUT, 
                                                        1.0f);

yafoc::platform_arduino::WiringRotorAnalogSensor rotor_sensor(TEMPERATURE, 
                                                            10, 
                                                            1013);
yafoc::platform_stm32::Stm32g431_Inverter inverter(12.0f);

using Current = yafoc::platform_arduino::WiringCurrentSensor;
using Rotor = yafoc::platform_arduino::WiringRotorAnalogSensor;
using Driver = yafoc::platform_stm32::Stm32g431_Inverter;

yafoc::ControllerFOC<Current,Rotor,Driver> motor_controller(11, 
                                                phase_sensors, 
                                                rotor_sensor, 
                                                inverter);

void setup() {
    motor_controller.InitializeAndAlignRotor();

    auto iq = yafoc::QCurrent(0.5f);
    auto id = yafoc::DCurrent(0.0f);
    motor_controller.SetTargetCurrent(id, iq);
}

void loop() {
    auto now = yafoc::TimeStamp((float)micros());
    motor_controller.RunControllerFOC(now);
}