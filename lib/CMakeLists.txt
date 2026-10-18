cmake_minimum_required(VERSION 3.18)
project(PhysicsEngine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SOURCES
    src/PhysicsWorld.cpp
    src/AABBTree.cpp
    src/Math.cpp
    src/Traits.cpp
    extern/Logger.cpp
)

add_library(sas_physics ${SOURCES})

#Just for 1 function
include(FetchContent)

set(TARGET_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(TARGET_HELLO_WORLD OFF CACHE BOOL "" FORCE)
set(TARGET_PERFORMANCE_TEST OFF CACHE BOOL "" FORCE)
set(TARGET_SAMPLES OFF CACHE BOOL "" FORCE)
set(TARGET_VIEWER OFF CACHE BOOL "" FORCE)

set(JPH_USE_VK OFF CACHE BOOL "" FORCE)
set(JPH_USE_DX12 OFF CACHE BOOL "" FORCE)
set(JPH_USE_MTL OFF CACHE BOOL "" FORCE)
set(JPH_USE_CPU_COMPUTE OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    JoltPhysics
    GIT_REPOSITORY https://github.com/jrouwe/JoltPhysics.git
    GIT_TAG master 
    GIT_SHALLOW TRUE
    SOURCE_SUBDIR Build 
)
FetchContent_MakeAvailable(JoltPhysics)


target_include_directories(sas_physics PUBLIC include extern)
target_link_libraries(sas_physics PRIVATE
    project_warnings
    project_sanitizers
    project_options
    Jolt
)

if(CMAKE_CUDA_COMPILER_LOADED)
    message(STATUS "Configuring sas_cuda_backend with: ${CMAKE_CUDA_COMPILER}")

    add_library(sas_cuda_backend STATIC
        src/CudaFunctions.cu
    )

    target_compile_options(sas_cuda_backend PRIVATE
        $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr -lineinfo>
    )

    set_target_properties(sas_cuda_backend PROPERTIES
        CUDA_STANDARD 20
        CUDA_STANDARD_REQUIRED ON
        CUDA_ARCHITECTURES "native"
    )

    target_include_directories(sas_cuda_backend PUBLIC include extern)

    target_link_libraries(sas_physics PRIVATE sas_cuda_backend)
    target_compile_definitions(sas_physics PUBLIC HAS_CUDA=1)
else()
    message(STATUS "CUDA NOT loaded in lib. Building CPU-only.")
    target_compile_definitions(sas_physics PUBLIC HAS_CUDA=0)
endif()
