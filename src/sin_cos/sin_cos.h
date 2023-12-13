#pragma once

#include <memory>

extern "C" {
extern void ours_arm_sin_cos_f32(float theta, float * pSinVal, float * pCosVal);
}

namespace yafoc {

inline void SinCos(float theta, float& sine, float& cosine) {
    ours_arm_sin_cos_f32(theta, std::addressof(sine), std::addressof(cosine));
}

}