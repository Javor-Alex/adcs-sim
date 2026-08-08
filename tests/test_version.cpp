#include <catch2/catch_test_macros.hpp>
#include <adcs/version.hpp>
#include <string>

TEST_CASE("version string is not empty") {
    REQUIRE(std::string(adcs::version()).size() > 0);
}
