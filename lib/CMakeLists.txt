cmake_minimum_required(VERSION 3.12)
project(PhysicsEngine)

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


add_library(sas_physics
    src/PhysicsWorld.cpp
    src/AABBTree.cpp
    src/Math.cpp
    src/Traits.cpp
    extern/Logger.cpp
)
target_include_directories(sas_physics PUBLIC include)
target_include_directories(sas_physics PUBLIC extern)

target_link_libraries(sas_physics PRIVATE project_warnings project_sanitizers project_options)