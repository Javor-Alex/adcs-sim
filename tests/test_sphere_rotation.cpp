#include <adcs/integrator.hpp>
#include <adcs/vec3.hpp>
#include <adcs/quaternion.hpp>
#include <adcs/rigid_body.hpp>
#include <Eigen/Dense>
#include <catch2/catch_test_macros.hpp>
#include <cmath>

TEST_CASE("sphere rotation") {
    int n = 100;
    double dt = 2*M_PI/n;
    double t = 0;
    double t_end = 6.3;

    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    adcs::Vec3 tau{0, 0, 0};

    adcs::RigidBody sphere{I, tau};

    adcs::Quaternion q{1, 0, 0, 0};

    adcs::Vec3 w{0, 0, 1};

    adcs::RigidBodyState state{q, w};

    bool w_stable = true;

    for (int i = 0; i < n; i++) {
        t = dt*i;
        state = adcs::rk4Step(state, t, dt, sphere);
        state.q.normalize();

        if (state.w != w) {
            w_stable = false;
        }
    }

    REQUIRE(w_stable);
    REQUIRE(state.q.isApprox(-adcs::Quaternion{}, 1e-7));
}
