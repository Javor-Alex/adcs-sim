#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <adcs/vec3.hpp>
#include <cmath>

using namespace adcs;

constexpr Vec3 a{-1.0, 3.0, 4.0};
constexpr Vec3 b{1.0, 2.0, 3.0};
constexpr Vec3 c{1e-12, 1e-13, 1e-14};
constexpr Vec3 d{1e12, 1e13, 1e14};
Vec3 a1;
constexpr Vec3 v0;
constexpr double s = 5;

TEST_CASE("cross antisymmetry") {
    REQUIRE(b.cross(a) == -a.cross(b));
    REQUIRE(c.cross(d) == -d.cross(c));
    REQUIRE(d.cross(b) == -b.cross(d));
}

TEST_CASE("orthogonality") {
    constexpr Vec3 bxa = b.cross(a);
    REQUIRE(std::fabs(b.dot(bxa)) < 1e-12 * b.norm() * bxa.norm());
    REQUIRE(std::fabs(a.dot(bxa)) < 1e-12 * a.norm() * bxa.norm());
}

TEST_CASE("normalization") {
    REQUIRE(b.normalized().norm() == Catch::Approx(1.0).epsilon(1e-12));
    REQUIRE(c.normalized().norm() == Catch::Approx(1.0).epsilon(1e-12));
    REQUIRE(d.normalized().norm() == Catch::Approx(1.0).epsilon(1e-12));
}

TEST_CASE("compound vs binary operations") {
    a1 = a;
    REQUIRE((a1 += b) == a + b);
    a1 = a;
    REQUIRE((a1 -= b) == a - b);
    a1 = a;
    REQUIRE((a1 *= s) == s*a);
    a1 = a;
    REQUIRE((a1 /= s) == a/s);
}

TEST_CASE("indexing") {
    REQUIRE(a[0] == a.x);
    REQUIRE(a[1] == a.y);
    REQUIRE(a[2] == a.z);

    a1 = a;

    a1[0] += 1;

    REQUIRE(a1[0] == a[0] + 1);

    const Vec3 a2 = a;

    REQUIRE(a2[0] == a[0]);
}

TEST_CASE("dot and cross product") {
    REQUIRE(a.dot(b) == b.dot(a));
    REQUIRE(a.dot(b + unitX) == a.dot(b) + a.dot(unitX));
    REQUIRE(a.cross(a) == v0);
    REQUIRE((s*a).dot(b) == s*a.dot(b));
}
