#pragma once

namespace yafoc {
namespace modulator {

#include "dq_transform.h"

inline void GetDqCurrents(float theta,
                        float iu,
                        float iv,
                        float iw,
                        float& iq,
                        float& id) {

    float phase_current_frame[3] = {iu, iv, iw};
    float ab_frame[2];
    dq_transforms::ClarkeTransform(phase_current_frame, ab_frame[0], ab_frame[1]);
    dq_transforms::ParkTransform(theta, ab_frame, id, iq);
}

inline void ModulateDqVoltages(float vbus_bias,
                            float theta,
                            float vd,
                            float vq,
                            float& vu,
                            float& vv,
                            float& vw) {
    float dq_frame[2] = {vd, vq};
    float ab_frame[2];

    dq_transforms::InverseParkTransform(theta, dq_frame, ab_frame[0], ab_frame[1]);
    dq_transforms::InverseClarkeTransform(ab_frame, vu,vv,vw);

    vu += vbus_bias;
    vv += vbus_bias;
    vw += vbus_bias;
}

}
}