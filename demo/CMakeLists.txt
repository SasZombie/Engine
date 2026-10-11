cmake_minimum_required(VERSION 3.12)
project(PhysicsEngine)

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(main2D demo2D.cpp)
# add_executable(main3D demo3D.cpp)

target_link_libraries(main2D PRIVATE
    sas_physics
    project_warnings
    project_options
    project_sanitizers
    raylib GL m pthread dl rt X11
)

# target_link_libraries(main3D PRIVATE
#     sas_physics
#     project_warnings
#     project_options
#     project_sanitizers
#     raylib GL m pthread dl rt X11
# )