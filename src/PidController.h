#pragma once

namespace yafoc {

struct PidControlGains {
    float m_kp{0.0f};
    float m_ki{0.0f};
    float m_kd{0.0f};

    float m_kp_sampled{};
    float m_ki_sampled{};
    float m_kd_sampled{};
    float m_b_n_coeffs[3]{};

    void MakePid2P2ZCoefficients(float sample_time_seconds) {
        m_kp_sampled = m_kp;
        m_ki_sampled = sample_time_seconds * 0.5f * m_ki;
        m_kd_sampled = (1.0f / sample_time_seconds) * m_kd;

        m_b_n_coeffs[0] = m_kp_sampled + m_ki_sampled + m_kd_sampled;
        m_b_n_coeffs[1] = -m_kp_sampled + m_ki_sampled + (2 * m_kd_sampled);
        m_b_n_coeffs[2] = m_kd_sampled;
    }
};

class PidController {
public:
    PidController(float limit_high, float limit_low);
    virtual ~PidController();
    void UpdateCoefficients(PidControlGains& gains);
    void Update(float reference, float measurement, float& out);

private:
    float m_x[3]{};
    float m_y{};
    float m_b[3]{1.0f, -1.0f, 0.0f};
    float m_limit_high;
    float m_limit_low;
};
}