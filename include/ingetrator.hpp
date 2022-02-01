#ifndef INTEGRATOR
#define INTEGRATOR

#include <cstddef>
#include <utility>

/**
* @brief Utility to simulate a single DoF dynamical system given a single input controller
*
* @arg timestep: simulation timestep in seconds
* @arg initial_velocity: initial system velocity
*/
struct Integrator
{
    Integrator(const double timestep, const double initial_velocity = 0.)
        : m_timestep(timestep),
          m_velocity(initial_velocity)
    {
    }
    /**
* @brief Simulate the dynamics for a single timestep
* @arg dyn: systems dynamics, must be invocable "double dyn(double velocity, double input)"
* @arg ctrl: controller, must be invocable "double ctrl(double (desired_velocity -
actual_velocity))"
* @return current simulation time and velocity as an std::pair<double time, double velocity>
*/
    template <class dyn_t, class ctrl_t>
    std::pair<double, double> step(dyn_t &&dyn, ctrl_t &&ctrl, const double desired_velocity)
    {
        m_time += m_timestep;
        m_velocity += m_timestep * dyn(m_velocity, ctrl(desired_velocity - m_velocity));
        return {m_time, m_velocity};
    }
    /**
* @brief Returns current simulation time
*/
    double get_time() const
    {
        return m_time;
    }
    /**
* @brief Returns current simulation velocity
*/
    double get_velocity() const
    {
        return m_velocity;
    }
    /**
* @brief Resets the simulator
* @arg initial_velocity: initial system velocity
*/
    void reset(const double initial_velocity = 0.)
    {
        m_time = 0.;
        m_velocity = initial_velocity;
    }

private:
    double m_time = 0.;
    double m_timestep;
    double m_velocity;
};

#endif