#pragma once 

namespace yafoc {
namespace rotor_sensor{
    template <class Derived>
    class RotorSensorInterface {
    public: 
        void SetCountToZero() {
            static_cast<Derived*>(this)->SetCountToZero();
        }

        float GetCpr() {
            return static_cast<Derived*>(this)->GetCpr();
        } 

        float ReadCounter() {
            return static_cast<Derived*>(this)->ReadCounter();
        }
    };

}    
}
