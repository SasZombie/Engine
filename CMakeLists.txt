cmake_minimum_required(VERSION 3.12)
project(PhysicsEngine)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_DEMO "Build the GUI demo app" ${PROJECT_IS_TOP_LEVEL})
option(BUILD_TESTS "Build tests" ${PROJECT_IS_TOP_LEVEL})

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

add_subdirectory(lib)

if(BUILD_DEMO AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/demo")
    add_subdirectory(demo)
endif()

if(BUILD_TESTS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests")
    add_subdirectory(tests)
endif()