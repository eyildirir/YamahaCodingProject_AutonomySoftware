#include "../include/ingetrator.hpp"
#include "../include/boatDynamics.hpp"
#include "../include/pidController.hpp"
#include "../include/gnuplot-iostream.h"

#include <iostream>

#define DT_DEF 0.010
#define T_SIM_MAX_DEF 10.0
#define V_INIT_DEF 0
#define V_DES_DEF 5

int main(int argc, char *argv[])
{
    double kp = atof(argv[1]);
    double kd = atof(argv[2]);
    double ki = atof(argv[3]);

    // 6 engine boat for tuning testing. The PID controls should mostly
    // be engine count agnostic, considering a lesser number of engines would
    // be a similar mathematical problem scaled down.
    const std::size_t n_engines = 6;

    // Default value passing for certain parameters.
    double dt = DT_DEF;
    double t_sim_max = T_SIM_MAX_DEF;
    double v_init = V_INIT_DEF;
    double v_des = V_DES_DEF;
    
    // Fall-through switch statement for parsing a variable number of args.
    // Unorthodox, but it works well, and it's easy toimplement and understand.
    switch (argc)
    {
        case 8:
            v_init = atof(argv[7]);
        case 7:
            t_sim_max = atof(argv[6]);
        case 6:
            dt = atof(argv[5]);
        case 5:
            v_des = atof(argv[4]);
            break;
    }

    // Set up the integrator, a 6 engine boat, pid controller, and state tracking.
    Integrator intr(dt, v_init);
    Boat<n_engines> boat;
    PIDController pid(kp, kd, ki, dt);
    std::pair<double, double> state(0, v_init);
    std::vector<std::pair<double, double>> states;
    
    // Bool for tracking the first pass at set point.
    double first_pass = -1;

    // Simulation loop.
    while (state.first <= t_sim_max)
    {
        state = intr.step<Boat<n_engines>, PIDController>(boat, pid, v_des);
        states.push_back(state);
        if (fabs(state.second-v_des) < v_des*0.01 && first_pass < 0)
        {
            std::cout << "Set Point First Pass: " << state.first << std::endl;
            first_pass = state.first;
        }
    }

    // Steady State Information.
    std::cout << "Steady State: " << states.back().second << std::endl;

    // Plotting functionality through GNUPlot.
    Gnuplot gp;
    gp << "set xrange [0:" << t_sim_max << "]\nset yrange [0:" << 2 * v_des + v_init << "]\n";
    gp << "plot" << gp.file1d(states) << "with lines title 'velocity " << kp << " " << kd << " "<< ki << " " << "'," << std::endl;

    return 0;
}