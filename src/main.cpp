#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <adcs/integrator.hpp>
#include <adcs/vec3.hpp>
#include <adcs/quaternion.hpp>
#include <adcs/rigid_body.hpp>
#include <Eigen/Dense>

int main(int argc, char* argv[]) {
    double dt = (argc > 1) ? std::stod(argv[1]) : 0.1;
    std::string out = (argc > 2) ? argv[2] : "data/test3.csv";
    
    double t = 0;
    double t_end = 60;

    Eigen::Matrix3d I;
    I << 1, 0, 0,
        0, 2, 0,
        0, 0, 3;

    adcs::Vec3 tau{0, 0, 0};

    adcs::RigidBody sat{I, tau};

    adcs::Quaternion q{1, 0, 0, 0};

    adcs::Vec3 w{1e-3, 1, 1e-3};

    adcs::RigidBodyState state{q, w};

    std::ofstream data(out);

    if (!data) {
        std::cerr << "could not open " << out << "\n";
        return 1;
    }

    data << std::setprecision(15);
    data << "t,qw,qx,qy,qz,wx,wy,wz\n";

    int n = std::lround(t_end / dt);
    for(int i = 0; i <= n; i++){
        t = dt*i;

        data << t << "," << state.q << "," << state.w << '\n';

        state = adcs::rk4Step(state, t, dt, sat);

        state.q.normalize();
    }

    return 0;
}
