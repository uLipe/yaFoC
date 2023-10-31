#pragma once

#include "trigonometry_utils.h"

namespace yafoc {
namespace dq_transforms {

inline void ClarkeTransform(float phase3frame[3], float& alpha , float& beta) {
    constexpr float k1 = 2.0/3.0f;
    constexpr float k2 = 1.0/3.0f;
    constexpr float sqrt3 = 1.73205080757f;
    constexpr float k3 = 2.0f / sqrt3;

    *alpha = k1 * phase3frame[0] - k2 * (phase3frame[1] - phase3frame[2]);
    *beta = k3  * (phase3frame[1] - phase3frame[2]);
}

inline void ParkTransform(float theta, float abframe[2], float& d , float& q) {

    d = abframe[0] * math_utils::Cosine(theta) +
         abframe[1] * math_utils::Sine(theta);

    q = abframe[1] * math_utils::Cosine(theta) -
         abframe[0] * math_utils::Sine(theta);
}

inline void InverseParkTransform(float theta, float dqframe[2], float& alpha, float& beta) {
    alpha = dqframe[0] * math_utils::Cosine(theta) -
            dqframe[1] * math_utils::Sine(theta);

    beta = dqframe[1] * math_utils::Cosine(theta) +
            dqframe[0] * math_utils::Sine(theta);
}

inline void InverseClarkeTransform(float abframe[2], float& a, float& b, float& c) {
    constexpr float sqrt3 = 1.73205080757f;

    a = abframe[0];
    b = (-abframe[0] + sqrt3 * abframe[1]) * 0.5f;
    c = (-abframe[0] - sqrt3 * abframe[1]) * 0.5f;
}

}
}