#pragma once 

namespace yafoc {
namespace current_sensor{

    struct PhaseCurrentsAmperes {
        float i_u;
        float i_v;
        float i_w;

        PhaseCurrentsAmperes(float iu, float iv, float iw) : 
            i_u(iu),i_v(iv),i_w(iw) {};
    };

    template <class Derived>
    class CurrentSensorInterface {
    public: 

        PhaseCurrentsAmperes GetPhaseCurrents() {
            return static_cast<Derived*>(this)->GetPhaseCurrents();
        }
    };
}    
}
