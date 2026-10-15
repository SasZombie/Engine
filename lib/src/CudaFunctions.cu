#include "CudaFunctions.hpp"

#include <cstdio>
#include <cuda_runtime.h>

__global__ void helloKernel() {
    printf("GPU executing from Physics Engine!\n");
}

void runGpuHello() {
    helloKernel<<<1, 1>>>();
    cudaDeviceSynchronize();
}