cmake_minimum_required(VERSION 3.12)
project(Parser)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/heads/main.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(Tests
    physics_tests.cpp
    collision_tests_circle.cpp
    collision_tests_box.cpp
    collision_tests_mixed.cpp
    angular_tests_box.cpp
)
target_link_libraries(Tests PRIVATE
    sas_physics
    project_warnings
    project_sanitizers
    GTest::gtest_main
)
