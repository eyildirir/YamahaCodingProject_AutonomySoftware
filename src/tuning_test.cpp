#include "../include/ingetrator.hpp"
#include "../include/boatDynamics.hpp"
#include "../include/pidController.hpp"
#include "../include/gnuplot-iostream.h"

#include <iostream>

#define DT_DEF 0.010
#define T_SIM_MAX_DEF 10.0
#define V_INIT_DEF 0

int main(int argc, char *argv[])
{
    double kp = atof(argv[1]);
    double kd = atof(argv[2]);
    double ki = atof(argv[3]);

    const std::size_t n_engines = 5;

    // const std::size_t n_engines = atof(argv[1]);

    double dt = DT_DEF;
    double t_sim_max = T_SIM_MAX_DEF;
    double v_init = V_INIT_DEF;
    

    if (argc <= 7 && argc >= 6)
    {
        dt = atof(argv[4]);
        t_sim_max = atof(argv[5]);
        if (argc = 8)
            v_init = atof(argv[6]);
    }

    double v_des = 5;

    Integrator intr(dt, 0);

    Boat<n_engines> boat;

    PIDController pid(kp, kd, ki, dt);
    
    std::pair<double, double> state(0, v_init);

    std::vector<std::pair<double, double>> states;
    
    double first_pass = -1;
    while (state.first <= t_sim_max)
    {
        // std::cout << state.first << " , " << t_sim_max << std::endl;
        // std::cin.get();
        state = intr.step<Boat<n_engines>, PIDController>(boat, pid, v_des);
        states.push_back(state);
        if (abs(state.second-v_des) < 0.05 && first_pass < 0)
        {
            std::cout << "Set Point First Pass: " << state.first << std::endl;
            first_pass = state.first;
        }
    }

    std::cout << "Steady State: " << states.back().second << std::endl;

    Gnuplot gp;

    gp << "set xrange [0:" << t_sim_max << "]\nset yrange [0:" << 2 * v_des << "]\n";

    gp << "plot" << gp.file1d(states) << "with lines title 'velocity'," << std::endl;

    return 0;
}