#pragma once 

namespace yafoc {
namespace conversions {

constexpr float FromMechanicalToElectricAngle(float mechanical, float pole_pairs) {
    return (mechanical * pole_pairs);
}

}
}