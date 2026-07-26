cmake_minimum_required(VERSION 3.12)
project(PhysicsEngine)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(main main.cpp)

target_link_libraries(main PRIVATE
    sas_physics
    project_warnings
    project_sanitizers
    raylib GL m pthread dl rt X11
)