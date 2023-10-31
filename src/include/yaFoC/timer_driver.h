#pragma once

namespace yafoc {
namespace timer_driver{

class TimerDriver {

using TimerCallback = std::function<void(float)>;

public:
    TimerDriver();
    virtual ~TimerDriver();
    virtual int Start(int expiration_time_us) = 0;
    virtual int Halt() = 0;
    virtual int Resume();
    virtual void BusyWait(int wait_us);
    virtual int RegisterCallback(TimerCallback cb);
protected:
    void *m_user_data;
    std::function<void(void*)> m_callback;
    int m_period_us{0};
    float m_dt{0};
};

}
}
