#pragma once

#include <cuda_runtime.h>

#include <cstdio>
#include <cstdlib>

// Fail fast; a failed CUDA call leaves the rest of the frame meaningless.
#define CUDA_CHECK(expr)                                                     \
  do {                                                                       \
    cudaError_t cuda_check_error = (expr);                                   \
    if (cuda_check_error != cudaSuccess) {                                   \
      std::fprintf(stderr, "CUDA error at %s:%d: %s: %s\n", __FILE__,        \
                   __LINE__, #expr, cudaGetErrorString(cuda_check_error));   \
      std::exit(EXIT_FAILURE);                                               \
    }                                                                        \
  } while (false)

