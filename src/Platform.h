#pragma once

namespace yafoc {

class Platform {
public:
    virtual void Trigger() {
        m_notification_flag = true;
    }

    virtual bool Poll() {
        return m_notification_flag;
    }

    virtual void Consume() {
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
