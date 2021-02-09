#pragma once 

namespace yafoc {
namespace inverter_driver{
    template <class Derived>
    class InverterInterface {
    public: 
        float GetVoltageSupply() {
            return static_cast<Derived*>(this)->GetVoltageSupply();
        } 

        void SetInverterVoltages(float u, float v, float w) {
            static_cast<Derived*>(this)->SetInverterVoltages(u,v,w);
        }
    };
}    
}
