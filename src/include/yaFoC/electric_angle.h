#pragma once 

extern "C" {
    #include <math.h>
}

namespace yafoc {
namespace conversions {

constexpr float FAST_PI = 3.14159265358f;
constexpr float FAST_2PI = FAST_PI * 2.0f;
constexpr float ALIGN_ANGLE_CONSTANT = (FAST_PI / 2.0f) - FAST_2PI; 

constexpr float FromMechanicalToElectricAngle(float mechanical, float pole_pairs) {
    return (mechanical * pole_pairs);
}

inline float NormalizeElectricalAngle(float full_angle) {
    
    float result =  fmod(full_angle, FAST_2PI);
    if(result > FAST_PI) {
        result -= FAST_2PI;  
    }
    
    return result;
}

}
}