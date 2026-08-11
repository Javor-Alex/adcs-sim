#pragma once

namespace adcs {

    template <typename F>
    double step(double x, double t, double dt, F f){

        //rk4 method
        double k1 = f(x, t);
        double k2 = f(x + dt*k1/2, t + dt/2);
        double k3 = f(x + dt*k2/2, t + dt/2);
        double k4 = f(x + dt*k3, t + dt);

        return x + dt*(k1 + 2*k2 + 2*k3 + k4)/6;
    }

}
