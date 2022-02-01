#ifndef BOATDYNAMICS
#define BOATDYNAMICS

#include <cstddef>
#include <algorithm>

template <std::size_t n_engines>
struct Boat
{
    double operator()(double velocity, double input)
    {
        const double acceleration = std::clamp(input, -1.0, 1.0) *
                                        n_engines - velocity;
        return acceleration;
    }
};

#endif