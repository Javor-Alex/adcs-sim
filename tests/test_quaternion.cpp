#include <catch2/catch_test_macros.hpp>
#include <adcs/quaternion.hpp>
#include <cmath>

constexpr double PI = 3.14159265358979323846; 

const adcs::Quaternion q = adcs::Quaternion{1.0, 2.0, 3.0, 4.0}.normalized();
const adcs::Quaternion r = adcs::Quaternion{5.0, -6.0, 7.0, -8.0}.normalized();
const adcs::Quaternion q_conj = q.conjugate();

constexpr adcs::Vec3 v{4.0, 5.0, 6.0};

TEST_CASE("quaternion normalization") {
    REQUIRE(std::fabs(q.norm() - 1) <= 1e-12);
    REQUIRE(std::fabs(r.norm() - 1) <= 1e-12);
}

TEST_CASE("unit tests") {
    REQUIRE((q * q_conj).isApprox(adcs::qIdentity));
    REQUIRE(q_conj.rotate(q.rotate(v)).isApprox(v));
    REQUIRE(q.rotate(q_conj.rotate(v)).isApprox(v));
}

TEST_CASE("quaternion compound vs binary operations") {

    adcs::Quaternion q1 = q;
    REQUIRE((q1 += r) == q + r);
    q1 = q;
    REQUIRE((q1 -= r) == q - r);

    double s = 2;

    q1 = q;
    REQUIRE((q1 *= s) == s*q);
    q1 = q;
    REQUIRE((q1 /= s) == q/s);
}

TEST_CASE("quaternion arithmetic operations") {
    REQUIRE(2*q == q + q);
    REQUIRE(2*r == r + r);
    REQUIRE(3*q == q + q + q);
}

TEST_CASE("rotation tests") {
    //90° about x
    adcs::Quaternion x = adcs::quatFromAngleAxis(PI/2, adcs::Vec3{1, 0, 0});
    adcs::Vec3 v1{0, 1, 0};
    REQUIRE(x.rotate(v1).isApprox(adcs::Vec3{0, 0, 1}));

    //90° about y
    adcs::Quaternion y = adcs::quatFromAngleAxis(PI/2, adcs::Vec3{0, 1, 0});
    adcs::Vec3 v2{1, 0, 0};
    REQUIRE(y.rotate(v2).isApprox(adcs::Vec3{0, 0, -1}));

    //90° about z
    adcs::Quaternion z = adcs::quatFromAngleAxis(PI/2, adcs::Vec3{0, 0, 1});
    adcs::Vec3 v3{1, 0, 0};
    REQUIRE(z.rotate(v3).isApprox(adcs::Vec3{0, 1, 0}));

    SECTION("composition test") {
        REQUIRE((q*r).rotate(v).isApprox(q.rotate(r.rotate(v))));
    }

    SECTION("length preservation") {
        REQUIRE(std::fabs(q.rotate(v).norm() - v.norm()) <= 1e-12);
        REQUIRE(std::fabs(r.rotate(v).norm() - v.norm()) <= 1e-12);
    }

    SECTION("q and -q rotating") {
        REQUIRE(q.rotate(v).isApprox((-q).rotate(v), 1e-14));
    }
    SECTION("matrix rotation") {
        REQUIRE(q.rotate(v).isApprox(q.toMatrix() * v));
        REQUIRE(r.rotate(v).isApprox(r.toMatrix() * v));
    }
}

TEST_CASE("round trips") {
    adcs::Quaternion tester = adcs::Quaternion{10.0, 1.0, 2.0, 3.0}.normalized();

    REQUIRE(adcs::quatFromMatrix(tester.toMatrix()).isApprox(tester));

    tester = adcs::Quaternion{1.0, 10.0, 2.0, 3.0}.normalized();
    REQUIRE(adcs::quatFromMatrix(tester.toMatrix()).isApprox(tester));

    tester = adcs::Quaternion{1.0, 1.0, 7.0, 3.0}.normalized();
    REQUIRE(adcs::quatFromMatrix(tester.toMatrix()).isApprox(tester));

    tester = adcs::Quaternion{1.0, 1.0, 4.0, 12.0}.normalized();
    REQUIRE(adcs::quatFromMatrix(tester.toMatrix()).isApprox(tester));

    tester = adcs::Quaternion{-1.0, 0.0, 0.0, 1.0}.normalized();

    REQUIRE((-tester).isApprox(adcs::quatFromAngleAxis(tester.angle(), tester.axis())));
    REQUIRE(q.isApprox(adcs::quatFromAngleAxis(q.angle(), q.axis())));
    REQUIRE(r.isApprox(adcs::quatFromAngleAxis(r.angle(), r.axis())));
}
