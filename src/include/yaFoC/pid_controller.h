#pragma once

namespace yafoc {
namespace controller {

class PidController {
    float previous_error{0.0f};
    float integrated_error{0.0f};

public:
    float kp{1.0f};
    float ki{0.0f};
    float kd{0.0f};
    float integrator_limit{1.0f};

    PidController() {
    }

    inline void Reset() {
        previous_error = 0.0f;
        integrated_error = 0.0;
    }

    inline float Update(float setpoint, float measured) {
        float error = setpoint.raw - measured.raw;
        float error_diff = error - previous_error;
        integrated_error += error;

        integrated_error = std::clamp(integrated_error,
                                    -integrator_limit,
                                    integrator_limit);

        float mv = kp * error +
                    (ki * integrated_error) +
                    (kd * error_diff);

        previous_error = error;

        return mv;
    }
};

}
}