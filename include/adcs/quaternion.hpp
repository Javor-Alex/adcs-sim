// CONVENTIONS: State: scalar-first (w,x,y,z); Hamilton product; q rotates body → inertial

#pragma once
#include <adcs/vec3.hpp>
#include <cassert>
#include <ostream>
#include <Eigen/Dense>
#include <cmath>

namespace adcs {

    struct Quaternion{
        double w = 1.0, x = 0.0, y = 0.0, z = 0.0;

        constexpr Quaternion() = default;

        constexpr Quaternion(double w_, double x_, double y_, double z_)
            : w(w_), x(x_), y(y_), z(z_) {}

        constexpr Quaternion(double scalar, const Vec3& v)
            : w(scalar), x(v.x), y(v.y), z(v.z) {}

        constexpr Vec3 vec() const {
            return Vec3{x, y, z};
        }

        constexpr Quaternion& operator+=(const Quaternion& rhs) { //geometically meaningless, just needed for rk4
            w += rhs.w;
            x += rhs.x;
            y += rhs.y;
            z += rhs.z;
            return *this;
        }

        constexpr Quaternion& operator-=(const Quaternion& rhs) {
            w -= rhs.w;
            x -= rhs.x;
            y -= rhs.y;
            z -= rhs.z;
            return *this;
        }

        constexpr Quaternion& operator*=(double scalar) {
            w *= scalar;
            x *= scalar;
            y *= scalar;
            z *= scalar;
            return *this;
        }

        constexpr Quaternion& operator/=(double scalar) {
            assert(scalar != 0 && "DIVISION BY ZERO");
            w /= scalar;
            x /= scalar;
            y /= scalar;
            z /= scalar;
            return *this;
        }

        constexpr Quaternion operator-() const {
            return Quaternion{-w, -x, -y, -z};
        }

        constexpr double normSquared() const {
            return w*w + x*x + y*y + z*z;
        }

        double norm() const {
            return std::sqrt(normSquared());
        }

        Quaternion& normalize() {
            double n = norm();
            assert(n != 0 && "DIVISION BY ZERO");
            return *this *= (1/n) ;
        }

        Quaternion normalized() const {
            double n = norm();
            assert(n != 0 && "DIVISION BY ZERO");
            return *this * (1/n);
        }

        constexpr Quaternion operator*(const Quaternion& rhs) const {
            Vec3 v = vec();
            Vec3 vr = rhs.vec();
            return Quaternion{w*rhs.w - v.dot(vr), w*vr + rhs.w*v + v.cross(vr)};
        }

        constexpr Quaternion conjugate() const {
            return Quaternion{w, -vec()};
        }

        constexpr Vec3 rotate(const Vec3& v) const {
            Quaternion v_quat{0, v};
            return ((*this)*(v_quat)*(conjugate())).vec();
        }

        constexpr bool isApprox(const Quaternion& q, double tol = 1e-12) const {
            return (*this - q).normSquared() <= tol*tol;
        }

        Vec3 axis() const {
            if (w < 0) {
                Quaternion neg_q{-w, -x, -y, -z};
                return neg_q.vec().normalized();
            }
            return vec().normalized();
        }

        double angle() const {
            if (w < 0) {
                Quaternion neg_q{-w, -x, -y, -z};
                return 2*std::atan2(neg_q.vec().norm(), neg_q.w);
            }
            return 2*std::atan2(vec().norm(), w);
        }
        
        Eigen::Matrix3d toMatrix() const {
            Eigen::Matrix3d R;

            assert((w != 0 || x != 0 || y != 0 || z != 0) && "CANT CONVERT ZERO QUATERNION");
            double s = 1/normSquared();

            R << 1 - 2*s*(y*y + z*z), 2*s*(x*y - z*w), 2*s*(x*z + y*w),
                    2*s*(x*y + z*w), 1 - 2*s*(x*x + z*z), 2*s*(y*z - x*w),
                    2*s*(x*z - y*w), 2*s*(y*z + x*w), 1 - 2*s*(x*x + y*y);

            return R;
        }

        //FREE FUNCTIONS

        friend std::ostream& operator<<(std::ostream& os, const Quaternion& q) {
            return os << q.w << ", " << q.x << ", " << q.y << ", " << q.z;
        }

        friend constexpr Quaternion operator+(Quaternion lhs, const Quaternion& rhs) { //geometically meaningless, just needed for rk4
            lhs += rhs;
            return lhs;
        }

        friend constexpr Quaternion operator-(Quaternion lhs, const Quaternion& rhs) {
            lhs -= rhs;
            return lhs;
        }

        friend constexpr Quaternion operator*(Quaternion lhs, double scalar) {
            lhs *= scalar;
            return lhs;
        }

        friend constexpr Quaternion operator*(double scalar, Quaternion rhs) {
            rhs *= scalar;
            return rhs;
        }

        friend constexpr Quaternion operator/(Quaternion lhs, double scalar) {
            lhs /= scalar;
            return lhs;
        }

        friend bool operator==(const Quaternion& lhs, const Quaternion& rhs) {
            return lhs.w == rhs.w && lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
        }

        friend bool operator!=(const Quaternion& lhs, const Quaternion& rhs) {
            return !(lhs == rhs);
        }
    };

    inline Quaternion quatFromAngleAxis(double angle, const Vec3& axis) {
            return Quaternion{std::cos(angle/2), std::sin(angle/2)*(axis.normalized())};
        }

    inline Quaternion quatFromMatrix(const Eigen::Matrix3d& R) {
        double w, x, y, z;

        assert(std::fabs(((R.transpose()*R) - Eigen::Matrix3d::Identity()).norm()) <= 1e-12 && std::fabs(R.determinant() - 1) <= 1e-12 && "NOT A ROTATION MATRIX"); 

        if (R(0, 0) + R(1, 1) + R(2, 2) >= 0) {
            w = 0.5*std::sqrt(1 + R(0, 0) + R(1, 1) + R(2, 2));
            x = (R(2, 1) - R(1, 2))/(4*w);
            y = (R(0, 2) - R(2, 0))/(4*w);
            z = (R(1, 0) - R(0, 1))/(4*w);
        }
        else if (R(0, 0) - R(1, 1) - R(2, 2) >= 0) {
            x = 0.5*std::sqrt(1 + R(0, 0) - R(1, 1) - R(2, 2));
            w = (R(2, 1) - R(1, 2))/(4*x);
            z = (R(0, 2) + R(2, 0))/(4*x);
            y = (R(1, 0) + R(0, 1))/(4*x);
        }
        else if ( -R(0, 0) + R(1, 1) - R(2, 2) >= 0) {
            y = 0.5*std::sqrt(1 - R(0, 0) + R(1, 1) - R(2, 2));
            z = (R(2, 1) + R(1, 2))/(4*y);
            w = (R(0, 2) - R(2, 0))/(4*y);
            x = (R(1, 0) + R(0, 1))/(4*y);
        }
        else {
            z = 0.5*std::sqrt(1 - R(0, 0) - R(1, 1) + R(2, 2));
            y = (R(2, 1) + R(1, 2))/(4*z);
            x = (R(0, 2) + R(2, 0))/(4*z);
            w = (R(1, 0) - R(0, 1))/(4*z);
        }

        return Quaternion{w, x, y, z};
    }

    inline constexpr Quaternion qIdentity{1.0, 0.0, 0.0, 0.0};

}
