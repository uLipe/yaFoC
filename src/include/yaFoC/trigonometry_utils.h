#pragma once 

namespace yafoc {
namespace math_utils {

inline float Sine(float x) {
    constexpr float FAST_PI = 3.14159265358f;
    constexpr float B = 4.0f / FAST_PI;
    constexpr float C = -4.0f / (FAST_PI * FAST_PI);
    constexpr float P = 0.225f;

    float y = B * x + C * x * (x < 0 ? -x : x);
    return P * (y * (y < 0 ? -y : y) - y) + y;
}

inline float Cosine(float x) {
    constexpr float FAST_PI = 3.14159265358f;
    constexpr float B = 4.0f / FAST_PI;
    constexpr float C = -4.0f / (FAST_PI * FAST_PI);
    constexpr float P = 0.225f;
    constexpr float D = FAST_PI/2.0f; 
    x = (x > 0) ? -x : x;
    x += D;

    return Sine(x);
}
    
}
}