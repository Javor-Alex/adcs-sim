#pragma once
#include <cassert>
#include <ostream>
#include <cmath>
#include <Eigen/Dense>

namespace adcs {

    struct Vec3{
        double x = 0.0, y = 0.0, z = 0.0;

        //MEMBER FUNCTIONS

        constexpr Vec3& operator+=(const Vec3& rhs) {
            x += rhs.x;
            y += rhs.y;
            z += rhs.z;
            return *this;
        }

        constexpr Vec3& operator-=(const Vec3& rhs) {
            x -= rhs.x;
            y -= rhs.y;
            z -= rhs.z;
            return *this;
        }

        constexpr Vec3& operator*=(double scalar) {
            x *= scalar;
            y *= scalar;
            z *= scalar;
            return *this;
        }

        constexpr Vec3& operator/=(double scalar) {
            assert(scalar != 0 && "DIVISION BY ZERO");
            x /= scalar;
            y /= scalar;
            z /= scalar;
            return *this;
        }

        constexpr Vec3 operator-() const {
            return Vec3{-x, -y, -z};
        }

        constexpr double& operator[](int i) {
            assert(i <= 2 && i >= 0 && "NO SUCH COMPONENT OF THE VECTOR EXISTS");
            if (i == 0) return x;
            if (i == 1) return y;
            return z;
        }

        constexpr const double& operator[](int i) const {
            assert(i <= 2 && i >= 0 && "NO SUCH COMPONENT OF THE VECTOR EXISTS");
            if (i == 0) return x;
            if (i == 1) return y;
            return z;
        }

        constexpr double dot(const Vec3& other) const {
            return x * other.x + y * other.y + z * other.z;
        }

        constexpr Vec3 cross(const Vec3& other) const {
            return Vec3{y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x};
        }

        constexpr double normSquared() const {
            return dot(*this);
        }

        double norm() const {
            return std::sqrt(normSquared());
        }

        Vec3& normalize() {
            return *this /= norm();
        }

        Vec3 normalized() const {
            return *this/norm();
        }

        constexpr bool isApprox(const Vec3& v, double tol = 1e-12) const {
            return (*this - v).normSquared() <= tol*tol;
        }

        //FREE FUNCTIONS

        friend constexpr Vec3 operator+(Vec3 lhs, const Vec3& rhs) {
            lhs += rhs;
            return lhs;
        }

        friend constexpr Vec3 operator-(Vec3 lhs, const Vec3& rhs) {
            lhs -= rhs;
            return lhs;
        }

        friend constexpr Vec3 operator*(double scalar, Vec3 rhs) {
            return rhs *= scalar;
        }

        friend constexpr Vec3 operator*(Vec3 lhs, double scalar) {
            lhs *= scalar;
            return lhs;
        }

        friend constexpr Vec3 operator/(Vec3 lhs, double scalar) {
            lhs /= scalar;
            return lhs;
        }

        friend constexpr bool operator==(const Vec3& lhs, const Vec3& rhs) {
            return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
        }

        friend constexpr bool operator!=(const Vec3& lhs, const Vec3& rhs) {
            return !(lhs == rhs);
        }

        friend std::ostream& operator<<(std::ostream& os, const Vec3& vec) {
            return os << vec.x << ", " << vec.y << ", " << vec.z;
        }

    };

    inline Eigen::Vector3d toEigen(const Vec3& v) {
        return Eigen::Vector3d{v.x, v.y, v.z};
    }

    inline Vec3 fromEigen(const Eigen::Vector3d& v) {
        return Vec3{v(0), v(1), v(2)};
    }

    inline Vec3 operator*(const Eigen::Matrix3d& R, const Vec3& v) {
        return fromEigen(R * toEigen(v));
    }

    inline constexpr Vec3 unitX{1.0, 0.0, 0.0};
    inline constexpr Vec3 unitY{0.0, 1.0, 0.0};
    inline constexpr Vec3 unitZ{0.0, 0.0, 1.0};

    static_assert(unitX.cross(unitY) == unitZ, "CROSS PRODUCT DOES NOT WORK");
    static_assert(unitY.cross(unitZ) == unitX, "CROSS PRODUCT DOES NOT WORK");
    static_assert(unitZ.cross(unitX) == unitY, "CROSS PRODUCT DOES NOT WORK");

}
