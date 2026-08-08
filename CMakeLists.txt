cmake_minimum_required(VERSION 3.20)
project(adcs_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(adcs_core
    src/version.cpp
)
target_include_directories(adcs_core PUBLIC include)

add_executable(sim src/main.cpp)
target_link_libraries(sim PRIVATE adcs_core)
