#pragma once

#include <cmath.h>

namespace yafoc {
namespace math_utils {

inline float Sine(float x) {
    return std::sinf(x);
}

inline float Cosine(float x) {
    return std::cosf(x);
}

}
}