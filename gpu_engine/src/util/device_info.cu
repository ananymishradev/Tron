#include "util/cuda_check.h"

#include <cuda_runtime.h>

#include <cstdio>

void print_device_info() {
  int device = 0;
  CUDA_CHECK(cudaGetDevice(&device));

  cudaDeviceProp properties{};
  CUDA_CHECK(cudaGetDeviceProperties(&properties, device));
  std::printf("device=%s\n", properties.name);
  std::printf("cc=%d.%d\n", properties.major, properties.minor);
  std::printf("sm_count=%d\n", properties.multiProcessorCount);
  std::printf("memory_mb=%zu\n",
              static_cast<std::size_t>(properties.totalGlobalMem / (1024 * 1024)));
}
