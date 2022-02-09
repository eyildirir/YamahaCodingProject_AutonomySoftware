#ifndef PIDCONTROLLER
#define PIDCONTROLLER

#include <cmath>
#include <list>
#include <numeric>

/**
* @brief A PID controller object to calculate a PID feedback loop with an error input.
*/
class PIDController
{
public:
    PIDController();
    PIDController(double _kp, double _kd, double _ki, double _dt);

    double operator()(double _err);

    void set_Kp(double _kp);
    void set_Kd(double _kd);
    void set_Ki(double _ki);

    void set_coeffs(double _kp, double _kd, double _ki);

private:
    double m_kp, m_ki, m_kd, m_dt, m_cv;
    std::size_t m_w_size;
    double m_errs[3];
};

/**
* @brief Constructor for PIDController object.
*/
PIDController::PIDController()
{
    m_kp = 0;
    m_kd = 0;
    m_ki = 0;
    m_dt = 0.10;
    m_cv = 0;
    std::fill(m_errs, m_errs+3, 0);
}

/**
* @brief Constructor for PIDController object.
*
* @arg _kp: Proportional Coefficient
* @arg _kd: Differential Coefficient
* @arg _ki: Integral Coefficient
* @arg _dt: Time Step Size
*/
PIDController::PIDController(double _kp, double _kd, double _ki, double _dt)
{
    m_kp = _kp;
    m_kd = _kd;
    m_ki = _ki;
    m_dt = _dt;
    m_cv = 0;
    std::fill(m_errs, m_errs+3, 0);
}

/**
* @brief Calculate PID feedback from most current error value.
*
* @arg _err: Error value (Setpoint vs. Process Variable)
* @return PID feedback control variable
*/
double PIDController::operator()(double _err)
{
    m_errs[2] = m_errs[1];
    m_errs[1] = m_errs[0];
    m_errs[0] = _err;

    double a0 = m_kp + m_ki * m_dt + m_kd/m_dt;
    double a1 = -m_kp -2*m_kd/m_dt;
    double a2 = m_kd/m_dt;

    m_cv = m_cv + a0*m_errs[0] + a1*m_errs[1] + a2*m_errs[2];
    return m_cv;
}

/**
* @brief Setter for proportional coefficient.
*
* @arg _kp: New Proportional Coefficient
*/
void PIDController::set_Kp(double _kp)
{
    m_kp = _kp;
}

/**
* @brief Setter for proportional coefficient.
*
* @arg _kp: New Differential Coefficient
*/
void PIDController::set_Kd(double _kd)
{
    m_kd = _kd;
}

/**
* @brief Setter for proportional coefficient.
*
* @arg _kp: New Integral Coefficient
*/
void PIDController::set_Ki(double _ki)
{
    m_ki = _ki;
}

/**
* @brief Setter for all coefficients.
*
* @arg _kp: New Proportional Coefficient
* @arg _kd: New Differential Coefficient
* @arg _ki: New Integral Coefficient
*/
void PIDController::set_coeffs(double _kp, double _kd, double _ki)
{
    m_kp = _kp;
    m_kd = _kd;
    m_ki = _ki;
}

#endif