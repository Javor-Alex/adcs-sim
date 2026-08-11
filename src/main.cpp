#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <adcs/integrator.hpp>

int main(int argc, char* argv[]) {
    double dt = (argc > 1) ? std::stod(argv[1]) : 0.1;
    std::string out = (argc > 2) ? argv[2] : "data/decay.csv";
    
    double t = 0;
    double t_end = 5;

    double x = 1;

    std::ofstream data(out);

    if (!data) {
        std::cerr << "could not open " << out << "\n";
        return 1;
    }

    data << std::setprecision(15);
    data << "t,x\n";

    auto f = [](double xi, double ti) { return -xi; };

    int n = std::lround(t_end / dt);
    for(int i = 0; i <= n; i++){
        t = dt*i;

        data << t << "," << x << '\n';

        x = adcs::step(x, t, dt, f);
    }

    return 0;
}
