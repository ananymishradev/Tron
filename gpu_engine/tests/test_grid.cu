#include "test_util.h"
#include "util/cuda_check.h"
#include "sim/grid.h"

#include <cuda_runtime.h>

#include <algorithm>
#include <cstdio>
#include <vector>

int main() {
  constexpr int count = 1000;
  std::vector<float4> host(count);
  for (int i = 0; i < count; ++i) {
    host[i] = {static_cast<float>(i % 10) * 0.25f,
               static_cast<float>((i / 10) % 10) * 0.25f,
               static_cast<float>(i / 100) * 0.25f, 1.0f};
  }
  float4* device = nullptr;
  CUDA_CHECK(cudaMalloc(&device, sizeof(float4) * count));
  CUDA_CHECK(cudaMemcpy(device, host.data(), sizeof(float4) * count,
                        cudaMemcpyHostToDevice));
  UniformGrid grid;
  launch_grid_setup(grid, device, count, 0.5f, 1 << 20, nullptr);
  launch_grid_build(device, count, grid, nullptr);
  CUDA_CHECK(cudaDeviceSynchronize());

  std::vector<int> sorted(count);
  std::vector<int> starts(grid.numCells + 1);
  CUDA_CHECK(cudaMemcpy(sorted.data(), grid.sortedIdx, sizeof(int) * count,
                        cudaMemcpyDeviceToHost));
  CUDA_CHECK(cudaMemcpy(starts.data(), grid.cellStart,
                        sizeof(int) * starts.size(), cudaMemcpyDeviceToHost));
  std::vector<int> seen(count, 0);
  for (int cell = 0; cell < grid.numCells; ++cell) {
    CHECK(starts[cell] <= starts[cell + 1]);
    for (int offset = starts[cell]; offset < starts[cell + 1]; ++offset) {
      const int index = sorted[offset];
      CHECK(index >= 0 && index < count);
      ++seen[index];
      CHECK(grid_linear_cell(make_float3(host[index].x, host[index].y, host[index].z),
                             grid) == cell);
    }
  }
  for (int value : seen) {
    CHECK(value == 1);
  }
  grid.free();

  std::vector<float4> huge = {{0.0f, 0.0f, 0.0f, 1.0f},
                              {1000.0f, 1000.0f, 1000.0f, 1.0f}};
  CUDA_CHECK(cudaMemcpy(device, huge.data(), sizeof(float4) * huge.size(),
                        cudaMemcpyHostToDevice));
  launch_grid_setup(grid, device, static_cast<int>(huge.size()), 0.01f, 128,
                    nullptr);
  CUDA_CHECK(cudaDeviceSynchronize());
  CHECK(grid_clamped(grid));
  CHECK(grid.numCells == 128);
  grid.free();
  CUDA_CHECK(cudaFree(device));
  return 0;
}
