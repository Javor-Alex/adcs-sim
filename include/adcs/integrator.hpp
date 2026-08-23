#pragma once

namespace adcs {

    template<typename State, typename Derivative>
    State rk4Step(const State& x, double t, double dt, Derivative f) {
        State k1 = f(x, t);
        State k2 = f(x + 0.5*dt*k1, t + 0.5*dt);
        State k3 = f(x + 0.5*dt*k2, t + 0.5*dt);
        State k4 = f(x + dt*k3, t + dt);
        return x + (dt/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);
}

}
