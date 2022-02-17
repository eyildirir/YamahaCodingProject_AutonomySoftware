#ifndef PIDCONTROLLER
#define PIDCONTROLLER

#include <cmath>
#include <list>
#include <numeric>
#include <tuple>

/**
 * @brief A PID controller object to calculate a PID feedback loop with an error input.
 */
class PIDController
{
public:
    PIDController();
    PIDController(double _kp, double _kd, double _ki, double _dt);

    double operator()(double _err);

    double get_Kp();
    double get_Kd();
    double get_Ki();

    std::tuple<double, double, double> get_coeffs();

    void set_Kp(double _kp);
    void set_Kd(double _kd);
    void set_Ki(double _ki);

    void set_coeffs(double _kp, double _kd, double _ki);
    void set_coeffs(std::tuple<double, double, double> _coeffs);

private:
    double m_kp, m_ki, m_kd, m_dt;
    std::size_t m_w_size;
    double m_errs[5];
    double m_cv[3];
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
    std::fill(m_cv, m_cv + 3, 0);
    std::fill(m_errs, m_errs + 5, 0);
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
    std::fill(m_cv, m_cv + 3, 0);
    std::fill(m_errs, m_errs + 3, 0);
}

/**
 * @brief Calculate PID feedback from most current error value.
 *
 * @arg _err: Error value (Setpoint vs. Process Variable)
 * @return double PID feedback control variable
 */
double PIDController::operator()(double _err)
{
    m_errs[4] = m_errs[3];
    m_errs[3] = m_errs[2];
    m_errs[2] = m_errs[1];
    m_errs[1] = m_errs[0];
    m_errs[0] = _err;

    m_cv[2] = m_cv[1];
    m_cv[1] = m_cv[0];

    // First Order Numerical Difference Approach for a discrete PID Regulator

    // double a0 = m_kp + m_ki * m_dt + m_kd/m_dt;
    // double a1 = -m_kp -2*m_kd/m_dt;
    // double a2 = m_kd/m_dt;
    // m_cv[0] = m_cv[0] + a0*m_errs[0] + a1*m_errs[1] + a2*m_errs[2];

    // Second Order Numerical Difference Approach for a discrete PID Regulator
    double a0 = 3 * m_kp + 2 * m_ki * m_dt + 4.5 * m_kd / m_dt;
    double a1 = -4 * m_kp - 12 * m_kd / m_dt;
    double a2 = m_kp + 11 * m_kd / m_dt;
    double a3 = -4 * m_kd / m_dt;
    double a4 = m_kd / (2 * m_dt);

    m_cv[0] = (a0 * m_errs[0] + a1 * m_errs[1] + a2 * m_errs[2] + a3 * m_errs[3] + a4 * m_errs[4] + 4 * m_cv[1] - m_cv[2]) / 3;
    return m_cv[0];
}

/**
 * @brief Return Proportional Coefficient
 * 
 * @return double Proportional Coefficient
 */
double PIDController::get_Kp()
{
    return m_kp;
}

/**
 * @brief Return Differential Coefficient
 * 
 * @return double Differential Coefficient
 */
double PIDController::get_Kd()
{
    return m_kd;
}

/**
 * @brief Returns Integral Coefficient.
 * 
 * @return double Integral Coefficient
 */
double PIDController::get_Ki()
{
    return m_ki;
}

/**
 * @brief Returns a tuple with all coefficients.
 * 
 * @return std::tuple<double, double, double> {kp, kd, ki}
 */
std::tuple<double, double, double> PIDController::get_coeffs()
{
    return std::make_tuple(m_kp, m_kd, m_ki);
}

/**
 * @brief Setter for Proportional Coefficient.
 *
 * @param _kp: New Proportional Coefficient
 */
void PIDController::set_Kp(double _kp)
{
    m_kp = _kp;
}

/**
 * @brief Setter for Differential Coefficient.
 *
 * @param _kp: New Differential Coefficient
 */
void PIDController::set_Kd(double _kd)
{
    m_kd = _kd;
}

/**
 * @brief Setter for Integral Coefficient.
 *
 * @param _kp: New Integral Coefficient
 */
void PIDController::set_Ki(double _ki)
{
    m_ki = _ki;
}

/**
 * @brief Setter for all coefficients.
 *
 * @param _kp: New Proportional Coefficient
 * @param _kd: New Differential Coefficient
 * @param _ki: New Integral Coefficient
 */
void PIDController::set_coeffs(double _kp, double _kd, double _ki)
{
    m_kp = _kp;
    m_kd = _kd;
    m_ki = _ki;
}

/**
 * @brief 
 * 
 * @param _coeffs A tuple of coefficients in the order kp, kd, ki.
 */
void PIDController::set_coeffs(std::tuple<double, double, double> _coeffs)
{
    m_kp = std::get<0>(_coeffs);
    m_kd = std::get<1>(_coeffs);
    m_ki = std::get<2>(_coeffs);
}

#endif