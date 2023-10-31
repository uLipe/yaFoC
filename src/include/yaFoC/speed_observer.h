#pragma once

namespace yafoc{
namespace speed_observer{

struct ObserverState {
    float m_states[2]{0,0};
    float m_input_position{0};
    float m_output_speed{0};
    float m_gain_p{10.0f};
    float m_gain_i{0.5f};
};

inline float ObserveSpeedFromPosition(ObserverState& o) {

}

};
}