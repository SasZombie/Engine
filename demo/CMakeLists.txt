cmake_minimum_required(VERSION 3.18)
project(PhysicsEngineDemo LANGUAGES CXX) 

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(raylib 6.2 QUIET)

if(NOT raylib_FOUND)
    include(FetchContent)
    
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
    
    FetchContent_Declare(
        raylib
        GIT_REPOSITORY https://github.com/raysan5/raylib.git
        GIT_TAG        6.2 
        GIT_SHALLOW    TRUE
    )
    FetchContent_MakeAvailable(raylib)
endif()

add_executable(main2D demo2D.cpp)
add_executable(main3D demo3D.cpp)

target_link_libraries(main2D PRIVATE
    sas_physics
    project_warnings
    project_options
    project_sanitizers
    raylib
)

target_link_libraries(main3D PRIVATE
    sas_physics
    project_warnings
    project_options
    project_sanitizers
    raylib
)