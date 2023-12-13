#pragma once

namespace yafoc {

class Platform {
public:
    virtual void SendNotification() {
        m_notification_flag = true;
    }

    virtual bool PollNotification() {
        return m_notification_flag;
    }

    virtual void ConsumeNotification() {
        m_notification_flag = false;
    }

    virtual void SetSamplingTime(float dt) {
        m_dt = dt;
    }

protected:
    bool m_notification_flag{false};
    float m_dt{0.0f};
};

}
