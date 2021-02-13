#pragma once 

namespace yafoc {
namespace rotor_sensor{
    template <class Derived>
    class RotorSensorInterface {
    public: 
        void SetCountToZero() {
            static_cast<Derived*>(this)->SetCountToZero();
        }

        float GetCountsPerRevolution() {
            return static_cast<Derived*>(this)->GetCountsPerRevolution();
        } 

        float ReadCounter() {
            return static_cast<Derived*>(this)->ReadCounter();
        }
    };

}    
}
