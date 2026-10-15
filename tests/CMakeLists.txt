cmake_minimum_required(VERSION 3.18)
project(Parser)

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/heads/main.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

include_directories(.)

add_executable(Tests2D
    2D/physics_tests.cpp
    2D/collision_tests_circle.cpp
    2D/collision_tests_box.cpp
    2D/collision_tests_mixed.cpp
    2D/angular_tests_box.cpp
    2D/performance_tests.cpp
)
add_executable(MathTests
    math_tests.cpp)

add_executable(PerformanceTests3D
    3D/performance_tests.cpp
)

add_executable(ProfilingTests3D1
    3D/Profiling/profiling_tests1.cpp
)
add_executable(ProfilingTests3D2
    3D/Profiling/profiling_tests2.cpp
)
add_executable(ProfilingTests3D3
    3D/Profiling/profiling_tests3.cpp
)
add_executable(Tests3D
    # 3D/physics_tests.cpp
    3D/collision_tests_circle.cpp
    3D/collision_tests_box.cpp
    3D/collision_tests_mixed.cpp
    3D/collision_tests_cilinder.cpp
    # 3D/collision_tests_GJK.cpp
    # 3D/angular_tests_box.cpp
    # 3D/performance_tests.cpp
)
target_link_libraries(Tests2D PRIVATE
    sas_physics
    project_warnings
    project_sanitizers
    project_options
    GTest::gtest_main
)

target_link_libraries(Tests3D PRIVATE
    sas_physics
    project_warnings
    project_sanitizers
    project_options
    GTest::gtest_main
)

target_link_libraries(MathTests PRIVATE
    sas_physics
    project_warnings
    project_sanitizers
    project_options
    GTest::gtest_main
)

target_link_libraries(PerformanceTests3D PRIVATE
    sas_physics
    project_warnings
    project_sanitizers
    project_options
    GTest::gtest_main
)

target_link_libraries(ProfilingTests3D1 PRIVATE
    sas_physics
    project_warnings
    project_sanitizers
    project_options
)

target_link_libraries(ProfilingTests3D2 PRIVATE
    sas_physics
    project_warnings
    project_sanitizers
    project_options
)

target_link_libraries(ProfilingTests3D3 PRIVATE
    sas_physics
    project_warnings
    project_sanitizers
    project_options
)