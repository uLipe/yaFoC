#include <Arduino.h>
#include <yaFoC.h>
#include <wiring_rotor_sensor.h>
#include <open_loop_phase_sensor.h>
#include <simulated_rotor_sensor.h>
#include "esp32_ledc_pwm_inverter.h"

constexpr int ENCODER_PIN = 2;
constexpr int PWM_U_PIN = 14;
constexpr int PWM_V_PIN = 13;
constexpr int PWM_W_PIN = 12;
constexpr int PWM_ENABLE = -1;


yafoc::platform_open_loop::OpenCurrentSensor phase_sensors;
yafoc::platform_simulator::SimulatedRotorSensor simu(1.0f);
yafoc::platform_arduino::WiringRotorAnalogSensor rotor_sensor(ENCODER_PIN,10.0f,4085.0f);
yafoc::platform_esp32::ESP32_Ledc_Inverter inverter(12.0f,
                                            PWM_U_PIN,
                                            PWM_V_PIN,
                                            PWM_W_PIN,
                                            PWM_ENABLE);

yafoc::FocTelemetry telem;

using Current = yafoc::platform_open_loop::OpenCurrentSensor;
using Rotor = yafoc::platform_arduino::WiringRotorAnalogSensor;
using Driver = yafoc::platform_esp32::ESP32_Ledc_Inverter;
using Simulator = yafoc::platform_simulator::SimulatedRotorSensor;

yafoc::ControllerFOC<Current,Simulator,Driver> motor_controller(11, 
                                                phase_sensors, 
                                                simu, 
                                                inverter);

void setup() {
    Serial.begin(1000000);
    motor_controller.InitializeAndAlignRotor();

    auto iq = yafoc::QCurrent(0.5f);
    auto id = yafoc::DCurrent(0.0f);
    motor_controller.SetTargetCurrent(id, iq);
}

void loop() {
    auto now = yafoc::TimeStamp((float)micros());
    motor_controller.RunControllerFOC(now);
    motor_controller.UpdateTelemetry(telem);

    Serial.print(telem.vabc[0]);
    Serial.print(" ");
    Serial.print(telem.vabc[1]);
    Serial.print(" ");
    Serial.print(telem.vabc[2]);
    Serial.print(" ");
    Serial.print(telem.vq);
    Serial.print(" ");
    Serial.println(telem.vd);
}