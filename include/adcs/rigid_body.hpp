# pragma once
#include <adcs/vec3.hpp>
#include <adcs/quaternion.hpp>
#include <Eigen/Dense>

namespace adcs {
    struct RigidBodyState {
        Quaternion q;
        Vec3 w;

        constexpr RigidBodyState& operator+=(const RigidBodyState& rhs) {
            q += rhs.q;
            w += rhs.w;
            return *this;
        }

        constexpr RigidBodyState& operator*=(double scalar) {
            q *= scalar;
            w *= scalar;
            return *this;
        }

        friend constexpr RigidBodyState operator*(double scalar, RigidBodyState rhs) {
            rhs *= scalar;
            return rhs;
        }

        friend constexpr RigidBodyState operator+(RigidBodyState lhs, const RigidBodyState& rhs) {
            lhs += rhs;
            return lhs;
        }
    };

    struct RigidBody {
    Eigen::Matrix3d I, I_inv;
    Vec3 tau;

    RigidBody(const Eigen::Matrix3d& I_,const Vec3& tau_)
            : I(I_), I_inv(I.inverse()), tau(tau_) {}

    RigidBodyState operator()(const RigidBodyState& s, double t) const {
        Vec3 w_dot = I_inv * (tau - s.w.cross(I * s.w));
        Quaternion q_dot = 0.5 * (s.q * Quaternion{0.0, s.w});
        return RigidBodyState{q_dot, w_dot};
    }
};
}
