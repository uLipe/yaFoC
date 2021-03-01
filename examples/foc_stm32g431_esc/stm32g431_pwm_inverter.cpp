#include <Arduino.h>
#include "stm32g431_pwm_inverter.h"

namespace yafoc {
namespace platform_stm32 {

constexpr int PWM_RESOLUTION = 12; // 12bit
constexpr float PWM_RANGE = 4095.0;// 2^12 -1 = 4095
constexpr int PWM_FREQUENCY = 25000 // 25khz

static void Stm32G431UpdatePhasePWm(int ulPin, uint32_t value, int resolution)
{
  PinName pin = digitalPinToPinName(ulPin);
  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(pin, PinMap_PWM);
  uint32_t index = get_timer_index(Instance);
  HardwareTimer *HT = (HardwareTimer *)(HardwareTimer_Handle[index]->__this);

  uint32_t channel = STM_PIN_CHANNEL(pinmap_function(pin, PinMap_PWM));
  HT->setCaptureCompare(channel, value, (TimerCompareFormat_t)resolution);
}

static HardwareTimer* ConfigStm32G431PwmInverter(uint32_t PWM_freq, 
                                                float dead_zone, 
                                                int pinA_h, 
                                                int pinA_l, 
                                                int pinB_h, 
                                                int pinB_l,
                                                int pinC_h, 
                                                int pinC_l)
{
  PinName uhPinName = digitalPinToPinName(pinA_h);
  PinName ulPinName = digitalPinToPinName(pinA_l);
  PinName vhPinName = digitalPinToPinName(pinB_h);
  PinName vlPinName = digitalPinToPinName(pinB_l);
  PinName whPinName = digitalPinToPinName(pinC_h);
  PinName wlPinName = digitalPinToPinName(pinC_l);

  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(uhPinName, PinMap_PWM);
 
  uint32_t index = get_timer_index(Instance);
  
  if (HardwareTimer_Handle[index] == NULL) {
    HardwareTimer_Handle[index]->__this = new HardwareTimer((TIM_TypeDef *)pinmap_peripheral(uhPinName, PinMap_PWM));
    HardwareTimer_Handle[index]->handle.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
    HAL_TIM_Base_Init(&(HardwareTimer_Handle[index]->handle));
    ((HardwareTimer *)HardwareTimer_Handle[index]->__this)->setOverflow(PWM_freq, HERTZ_FORMAT);  
  }
  HardwareTimer *HT = (HardwareTimer *)(HardwareTimer_Handle[index]->__this);
     
  HT->setMode(STM_PIN_CHANNEL(pinmap_function(uhPinName, PinMap_PWM)), TIMER_OUTPUT_COMPARE_PWM1, uhPinName);
  HT->setMode(STM_PIN_CHANNEL(pinmap_function(ulPinName, PinMap_PWM)), TIMER_OUTPUT_COMPARE_PWM1, ulPinName);
  HT->setMode(STM_PIN_CHANNEL(pinmap_function(vhPinName, PinMap_PWM)), TIMER_OUTPUT_COMPARE_PWM1, vhPinName);
  HT->setMode(STM_PIN_CHANNEL(pinmap_function(vlPinName, PinMap_PWM)), TIMER_OUTPUT_COMPARE_PWM1, vlPinName);
  HT->setMode(STM_PIN_CHANNEL(pinmap_function(whPinName, PinMap_PWM)), TIMER_OUTPUT_COMPARE_PWM1, whPinName);
  HT->setMode(STM_PIN_CHANNEL(pinmap_function(wlPinName, PinMap_PWM)), TIMER_OUTPUT_COMPARE_PWM1, wlPinName);

  // dead time is set in nanoseconds
  uint32_t dead_time_ns = (float)(1e9/PWM_freq)*dead_zone;
  uint32_t dead_time = __LL_TIM_CALC_DEADTIME(SystemCoreClock, LL_TIM_GetClockDivision(HT->getHandle()->Instance), dead_time_ns);
  LL_TIM_OC_SetDeadTime(HT->getHandle()->Instance, dead_time); // deadtime is non linear!
  LL_TIM_CC_EnableChannel(HT->getHandle()->Instance, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N | LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);

  HT->pause();
  HT->refresh();
  HT->resume();  
  return HT;
}

Stm32g431_Inverter::Stm32g431_Inverter( float link_voltage) : 
                                    dc_link_voltage(link_voltage) {
    
    voltage_to_duty_scale = PWM_RANGE / dc_link_voltage;

    pwm_uh = PHASE_UH;
    pwm_ul = PHASE_UL;
    pwm_vh = PHASE_VH;
    pwm_vl = PHASE_VL;
    pwm_wh = PHASE_WH;
    pwm_wl = PHASE_WL;

    ConfigStm32G431PwmInverter(PWM_FREQUENCY * 2,
                               100.0f, //100 nanoseconds
                               pwm_uh,
                               pwm_ul,
                               pwm_vh,
                               pwm_vl,
                               pwm_wh,
                               pwm_wl);

    Stm32G431UpdatePhasePWm(pwm_uh, 0, PWM_RESOLUTION);
    Stm32G431UpdatePhasePWm(pwm_vh, 0, PWM_RESOLUTION);
    Stm32G431UpdatePhasePWm(pwm_wh, 0, PWM_RESOLUTION);
}

Stm32g431_Inverter::~Stm32g431_Inverter() {
    Stm32G431UpdatePhasePWm(pwm_uh, 0, PWM_RESOLUTION);
    Stm32G431UpdatePhasePWm(pwm_vh, 0, PWM_RESOLUTION);
    Stm32G431UpdatePhasePWm(pwm_wh, 0, PWM_RESOLUTION);
}

float Stm32g431_Inverter::GetVoltageSupply() {
    return dc_link_voltage;
}

void Stm32g431_Inverter::SetInverterVoltages(float u, float v, float w) {
    Stm32G431UpdatePhasePWm(pwm_uh, u * voltage_to_duty_scale, PWM_RESOLUTION);
    Stm32G431UpdatePhasePWm(pwm_vh, v * voltage_to_duty_scale, PWM_RESOLUTION);
    Stm32G431UpdatePhasePWm(pwm_wh, w * voltage_to_duty_scale, PWM_RESOLUTION);
}

}
}
