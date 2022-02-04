#ifndef PIDCONTROL
#define PIDCONTROL

#include <cmath>
#include <list>
#include <numeric>

struct PID
{
    double errors[10] = {0};

    void update_errs(double _err)
    {
        for (int i = 9; i > 0; i--)
        {
            errors[i] = errors[i - 1];
        }
        errors[0] = _err;
    }

    double operator()(double error)
    {

        const double fp = kp * errors[0];
        const double fd = kd * (3 * errors[0] - 4 * errors[1] + errors[2]) / 2;
        const double fi = ki * std::accumulate(std::begin(errors), std::end(errors), 0);

        return fp + fd + fi;
    }
};

#endif