#include <Arduino.h>
#include "esp32_ledc_pwm_inverter.h"

namespace yafoc{
namespace platform_esp32 {

constexpr int FOC_PWM_RESOLUTION = 8; 
constexpr float FOC_PWM_RANGE = 255.0f;
constexpr int FOC_PWM_FREQUENCY = 30000;

constexpr int ESP_PWM_CHAN_U = 0;
constexpr int ESP_PWM_CHAN_V = 1;
constexpr int ESP_PWM_CHAN_W = 2;


ESP32_Ledc_Inverter::ESP32_Ledc_Inverter(float link_voltage, 
                                        int pin_u, 
                                        int pin_v, 
                                        int pin_w, 
                                        int enable) : 
                                    dc_link_voltage(link_voltage),
                                    pwm_uh(pin_u),
                                    pwm_vh(pin_v),
                                    pwm_wh(pin_w),
                                    pin_enable(enable) {
    
  voltage_to_duty_scale = FOC_PWM_RANGE / dc_link_voltage;

  ledcSetup(ESP_PWM_CHAN_U, FOC_PWM_FREQUENCY, FOC_PWM_RESOLUTION);
  ledcSetup(ESP_PWM_CHAN_V, FOC_PWM_FREQUENCY, FOC_PWM_RESOLUTION);
  ledcSetup(ESP_PWM_CHAN_W, FOC_PWM_FREQUENCY, FOC_PWM_RESOLUTION);

  ledcAttachPin(pwm_uh, ESP_PWM_CHAN_U);
  ledcAttachPin(pwm_vh, ESP_PWM_CHAN_V);
  ledcAttachPin(pwm_wh, ESP_PWM_CHAN_W);

  ledcWrite(ESP_PWM_CHAN_U, 0);  
  ledcWrite(ESP_PWM_CHAN_V, 0);  
  ledcWrite(ESP_PWM_CHAN_W, 0);  

  if(pin_enable > 0) {
    digitalWrite(pin_enable, HIGH);
  }
}

ESP32_Ledc_Inverter::~ESP32_Ledc_Inverter() {
  if(pin_enable > 0) {
    digitalWrite(pin_enable, LOW);
  }

  ledcWrite(ESP_PWM_CHAN_U, 0);  
  ledcWrite(ESP_PWM_CHAN_V, 0);  
  ledcWrite(ESP_PWM_CHAN_W, 0);  
}

float ESP32_Ledc_Inverter::GetVoltageSupply() {
  return dc_link_voltage;
}

void ESP32_Ledc_Inverter::SetInverterVoltages(float u, float v, float w) {

  if(u > dc_link_voltage) {
    u = dc_link_voltage;
  } else if (u < 0.0f) {
    u = 0.0f;
  }

  if(v > dc_link_voltage) {
    v = dc_link_voltage;
  } else if (v < 0.0f) {
    v = 0.0f;
  }

  if(w > dc_link_voltage) {
    w = dc_link_voltage;
  } else if (w < 0.0f) {
    w = 0.0f;
  }


  ledcWrite(ESP_PWM_CHAN_U, u * voltage_to_duty_scale);
  ledcWrite(ESP_PWM_CHAN_V, v * voltage_to_duty_scale);
  ledcWrite(ESP_PWM_CHAN_W, w * voltage_to_duty_scale);
}

}
}