#include "../include/ingetrator.hpp"
#include "../include/boatDynamics.hpp"
#include "../include/pidController.hpp"
#include "../include/gnuplot-iostream.h"

#include <iostream>
#include <variant>

// Predetermined Constants for Compile-Time.
#define DT 0.010
#define T_SIM_MAX 10.0
#define V_INIT 0
#define V_DES 5
#define KP 20
#define KD 0.01
#define KI 1

int main(int argc, char *argv[])
{
    // Take in engine count.
    std::size_t n_engines = atoi(argv[1]);

    // Create an Integrator with the predetermined time step and intial speed.
    Integrator intr(DT, V_INIT);

    // Generate a range of boats at compile time to not deal with 
    // the const parameter limitation of templates in C++
    Boat<1> boat1;
    Boat<2> boat2;
    Boat<3> boat3;
    Boat<4> boat4;
    Boat<5> boat5;
    Boat<6> boat6;
    
    // Create a PIDController object with the predetermined coeffs and time step.
    PIDController pid(KP, KD, KI, DT);

    // Initial state of the boat.
    std::pair<double, double> state(0, V_INIT);

    // Create a sequence of states to store the boat state over time.
    std::vector<std::pair<double, double>> states;

    while (state.first <= T_SIM_MAX)
    {
        // A very ugly way to accomodate for up to 6 engines...
        // If I had more time to spare, I may have been able to figure out a
        // neater way through std::variants like the Bonus Question suggested.
        switch (n_engines)
        {
        case 1:
            state = intr.step<Boat<1>, PIDController>(boat1, pid, V_DES);
            break;
        case 2:
            state = intr.step<Boat<2>, PIDController>(boat2, pid, V_DES);
            break;
        case 3:
            state = intr.step<Boat<3>, PIDController>(boat3, pid, V_DES);
            break;
        case 4:
            state = intr.step<Boat<4>, PIDController>(boat4, pid, V_DES);
            break;
        case 5:
            state = intr.step<Boat<5>, PIDController>(boat5, pid, V_DES);
            break;
        default:
            state = intr.step<Boat<6>, PIDController>(boat6, pid, V_DES);
            break;
        }

        // Store the boat state.
        states.push_back(state);
    }

    // Plotting functionality through GNUPlot.
    Gnuplot gp;
    gp << "set xrange [0:" << T_SIM_MAX << "]\nset yrange [0:" << 2 * V_DES << "]\n";
    gp << "plot" << gp.file1d(states) << "with lines title 'velocity " << KP << " " << KD << " " << KI << " "
       << "'," << std::endl;

    return 0;
}