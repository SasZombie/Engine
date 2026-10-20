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


target_include_directories(sas_physics PUBLIC include extern)
target_link_libraries(sas_physics PRIVATE
    project_warnings
    project_sanitizers
    project_options
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
