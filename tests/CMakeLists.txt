add_executable(tests test_version.cpp test_integrator.cpp test_vec3.cpp test_quaternion.cpp)
target_link_libraries(tests PRIVATE adcs_core Catch2::Catch2WithMain)

list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)
catch_discover_tests(tests)
