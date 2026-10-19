#include "test_util.h"
#include "util/cuda_check.h"

int main() {
  void* device_memory = nullptr;
  CUDA_CHECK(cudaMalloc(&device_memory, 1 << 20));
  CUDA_CHECK(cudaMemset(device_memory, 0xAB, 1 << 20));

  unsigned char first = 0;
  CUDA_CHECK(cudaMemcpy(&first, device_memory, 1, cudaMemcpyDeviceToHost));
  CHECK(first == 0xAB);

  CUDA_CHECK(cudaFree(device_memory));
  return 0;
}
