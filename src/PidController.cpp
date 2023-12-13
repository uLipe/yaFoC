#include <PidController.h>

using namespace std;
using namespace yafoc;

PidController::PidController(float limit_high, float limit_low) :
m_limit_high(limit_high), m_limit_low(limit_low) {}

PidController::~PidController() {}

void PidController::UpdateCoefficients(PidControlGains& gains) {
    m_b[0] = gains.m_b_n_coeffs[0];
    m_b[1] = gains.m_b_n_coeffs[1];
    m_b[2] = gains.m_b_n_coeffs[2];
}

void PidController::Update(float reference, float measurement, float& out) {
    m_x[0] = reference - measurement;
    float u = m_y + m_b[0] * m_x[0] +
            m_b[1] * m_x[1] + m_b[2] * m_x[2];

    if(u > m_limit_high) {
        u = m_limit_high;
    } else if(u < m_limit_low) {
        u = m_limit_low;
    }

    m_y = -u;
    m_x[2] = m_x[1];
    m_x[1] = m_x[0];
}
