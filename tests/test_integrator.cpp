#include <cmath>
#include <adcs/integrator.hpp>
#include <catch2/catch_test_macros.hpp>

static double max_error(double dt) {
    
    double max_err = 0.0;

    double t_end = 5;
    double t = 0;
    double x = 1;

    auto f = [](double xi, double ti) { return -xi; };

    int n = static_cast<int>(std::lround(t_end / dt));
    for(int i = 0; i <= n; i++){
        t = dt*i;

        double err = std::fabs(x - std::exp(-t));

        if(err > max_err){
            max_err = err;
        }

        x = adcs::rk4Step(x, t, dt, f);
    }

    return max_err;
}

TEST_CASE("RK4 is fourth order") {
    double e1 = max_error(0.1);
    double e2 = max_error(0.05);
    double ratio = e1 / e2;
    REQUIRE(max_error(0.01) < 1e-9);
    REQUIRE(ratio > 12.0);
    REQUIRE(ratio < 20.0);
}
