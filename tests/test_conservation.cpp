#include <catch2/catch_test_macros.hpp>
#include <adcs/integrator.hpp>
#include <adcs/vec3.hpp>
#include <adcs/quaternion.hpp>
#include <adcs/rigid_body.hpp>
#include <cmath>
#include <Eigen/Dense>

TEST_CASE("|L|, KE, q unit, conservation") {
    double dt = 0.0125;
    double t = 0;
    double t_end = 1000;

    Eigen::Matrix3d I;
    I << 1, 0, 0,
        0, 2, 0,
        0, 0, 3;

    adcs::Vec3 tau{0, 0, 0};

    adcs::RigidBody sat{I, tau};

    adcs::Quaternion q{1, 0, 0, 0};

    adcs::Vec3 w{1e-3, 1, 1e-3};

    adcs::RigidBodyState state{q, w};

    bool L_conservation = true;
    bool KE_conservation = true;
    bool q_unit_conservation = true;

    adcs::Vec3 L = I * state.w;
    double KE = (0.5*state.w).dot((I*state.w));

    int n = std::lround(t_end / dt);
    for(int i = 0; i <= n; i++){
        t = dt*i;

        if ((L - state.q.rotate(I * state.w)).norm() > 1e-9) {
            L_conservation = false;
        }
        if (std::fabs(KE - (0.5*state.w).dot((I*state.w))) > 1e-10) {
            KE_conservation = false;
        }
        if (std::fabs(state.q.norm() - 1) > 1e-10) {
            q_unit_conservation = false;
        }

        state = adcs::rk4Step(state, t, dt, sat);
    }

    CHECK(L_conservation);
    CHECK(KE_conservation);
    CHECK(q_unit_conservation);
}
