cmake_minimum_required(VERSION 3.12)
project(PhysicsEngine)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(project_warnings INTERFACE)
add_library(project_sanitizers INTERFACE)


if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(project_warnings INTERFACE
        -Wall -Wextra -Wformat-nonliteral -Wcast-align -Wpointer-arith
        -Wmissing-declarations -Winline -Wundef -Wcast-qual -Wshadow
        -Wwrite-strings -Wno-unused-parameter -Wfloat-equal
    )
    target_compile_options(project_sanitizers INTERFACE
        -fsanitize=undefined -fsanitize=address -fsanitize=leak -g
    )
    target_link_options(project_sanitizers INTERFACE
        -fsanitize=undefined -fsanitize=address -fsanitize=leak
    )
else()
    target_compile_options(project_warnings INTERFACE -O3)
endif()

add_library(sas_physics
    src/PhysicsWorld.cpp
    src/AABBTree.cpp
)
target_include_directories(sas_physics PUBLIC include)

target_link_libraries(sas_physics PRIVATE project_warnings project_sanitizers)

add_executable(main src/main.cpp)

target_link_libraries(main PRIVATE
    sas_physics
    project_warnings
    project_sanitizers
    raylib GL m pthread dl rt X11
)

include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/heads/main.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(Tests
    tests/physics_tests.cpp
    tests/collision_tests.cpp
)
target_link_libraries(Tests PRIVATE
    sas_physics
    project_warnings
    project_sanitizers
    GTest::gtest_main
)
