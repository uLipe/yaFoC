#pragma once 

namespace yafoc {
namespace controller {

struct Setpoint {
    float raw;
    Setpoint(float value) : raw(value) {}; 
};

struct Measured {
    float raw;
    Measured(float value) : raw(value) {}; 
};

class PidController {
    float previous_error;
    float integrated_error;
    
public:
    float kp;
    float ki;
    float kd;
    float integrator_limit;

    PidController() {
        kp = 1.0f;
        previous_error = 0.0f;
        integrated_error = 0.0;
        integrator_limit = 1.0f;
    }

    inline void Reset() {
        previous_error = 0.0f;
        integrated_error = 0.0;
    }

    inline float Update(Setpoint& setpoint, Measured& measured, float dt) {
        float error = setpoint.raw - measured.raw;
        float error_diff = error - previous_error;
        integrated_error += error;

        if(integrated_error > integrator_limit) {
            integrated_error = integrator_limit;
        } else if (integrated_error < -integrator_limit) {
            integrated_error = -integrator_limit;
        }

        float mv = kp * error + 
                    (ki * dt * integrated_error) + 
                    ((kd * error_diff) / dt);

        previous_error = error;

        return mv;
    }
};

}
}