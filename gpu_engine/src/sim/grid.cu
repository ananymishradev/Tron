#include "sim/grid.h"

#include "util/cuda_check.h"

#include <cub/cub.cuh>
#include <cuda_runtime.h>

#include <algorithm>
#include <climits>
#include <cstdio>
#include <vector>

namespace {

int linear_cell_host(float3 position, const UniformGrid& grid) {
  const int x = std::clamp(static_cast<int>((position.x - grid.min.x) / grid.cellSize),
                           0, grid.dims.x - 1);
  const int y = std::clamp(static_cast<int>((position.y - grid.min.y) / grid.cellSize),
                           0, grid.dims.y - 1);
  const int z = std::clamp(static_cast<int>((position.z - grid.min.z) / grid.cellSize),
                           0, grid.dims.z - 1);
  const long long cell =
      static_cast<long long>(z) * grid.dims.y * grid.dims.x +
      static_cast<long long>(y) * grid.dims.x + x;
  return static_cast<int>(std::min<long long>(cell, grid.numCells - 1));
}

__device__ int linear_cell_device(float4 position, const UniformGrid grid) {
  const int x = max(0, min(grid.dims.x - 1,
                           static_cast<int>((position.x - grid.min.x) /
                                             grid.cellSize)));
  const int y = max(0, min(grid.dims.y - 1,
                           static_cast<int>((position.y - grid.min.y) /
                                             grid.cellSize)));
  const int z = max(0, min(grid.dims.z - 1,
                           static_cast<int>((position.z - grid.min.z) /
                                             grid.cellSize)));
  const long long cell =
      static_cast<long long>(z) * grid.dims.y * grid.dims.x +
      static_cast<long long>(y) * grid.dims.x + x;
  return static_cast<int>(min(cell, static_cast<long long>(grid.numCells - 1)));
}

__global__ void clear_counts_kernel(int* counts, int num_cells) {
  const int index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index < num_cells) {
    counts[index] = 0;
  }
}

__global__ void count_cells_kernel(const float4* positions, int count,
                                   UniformGrid grid) {
  const int index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index < count) {
    atomicAdd(&grid.cellCount[linear_cell_device(positions[index], grid)], 1);
  }
}

__global__ void place_cells_kernel(const float4* positions, int count,
                                   UniformGrid grid) {
  const int index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index < count) {
    const int cell = linear_cell_device(positions[index], grid);
    // Keep the original index; collisions use it to read positions and velocities.
    grid.sortedIdx[atomicAdd(&grid.cellCursor[cell], 1)] = index;
  }
}

}  // namespace

int grid_cell_count(int3 dims) {
  const long long product = static_cast<long long>(dims.x) * dims.y * dims.z;
  if (product <= 0) {
    return 0;
  }
  return static_cast<int>(std::min<long long>(product, INT_MAX));
}

bool grid_clamped(const UniformGrid& grid) { return grid.clamped; }

int grid_linear_cell(float3 position, const UniformGrid& grid) {
  return linear_cell_host(position, grid);
}

void UniformGrid::alloc(int max_cells, int n) {
  free();
  allocatedCells = std::max(1, max_cells);
  allocatedParticles = std::max(0, n);
  CUDA_CHECK(cudaMalloc(&cellCount, sizeof(int) * allocatedCells));
  CUDA_CHECK(cudaMalloc(&cellStart, sizeof(int) * (allocatedCells + 1)));
  CUDA_CHECK(cudaMalloc(&cellCursor, sizeof(int) * allocatedCells));
  if (allocatedParticles > 0) {
    CUDA_CHECK(cudaMalloc(&sortedIdx, sizeof(int) * allocatedParticles));
  }
}

void UniformGrid::free() {
  cudaFree(cellCount);
  cudaFree(cellStart);
  cudaFree(cellCursor);
  cudaFree(sortedIdx);
  cudaFree(scanStorage);
  cellCount = nullptr;
  cellStart = nullptr;
  cellCursor = nullptr;
  sortedIdx = nullptr;
  scanStorage = nullptr;
  scanStorageBytes = 0;
  allocatedCells = 0;
  allocatedParticles = 0;
  numCells = 0;
}

void launch_grid_setup(UniformGrid& grid, const float4* pos, int n,
                       float cell_size, int max_cells, cudaStream_t stream) {
  if (n <= 0 || max_cells <= 0) {
    grid.numCells = 0;
    return;
  }

  std::vector<float4> positions(n);
  CUDA_CHECK(cudaMemcpyAsync(positions.data(), pos, sizeof(float4) * n,
                             cudaMemcpyDeviceToHost, stream));
  CUDA_CHECK(cudaStreamSynchronize(stream));
  Vec3 minimum{positions[0].x, positions[0].y, positions[0].z};
  Vec3 maximum = minimum;
  for (const float4& value : positions) {
    minimum.x = std::min(minimum.x, value.x);
    minimum.y = std::min(minimum.y, value.y);
    minimum.z = std::min(minimum.z, value.z);
    maximum.x = std::max(maximum.x, value.x);
    maximum.y = std::max(maximum.y, value.y);
    maximum.z = std::max(maximum.z, value.z);
  }
  cell_size = std::max(cell_size, 1e-6f);
  const int3 dimensions{
      std::max(1, static_cast<int>(std::ceil((maximum.x - minimum.x) / cell_size)) + 1),
      std::max(1, static_cast<int>(std::ceil((maximum.y - minimum.y) / cell_size)) + 1),
      std::max(1, static_cast<int>(std::ceil((maximum.z - minimum.z) / cell_size)) + 1)};
  // Clamp before allocation so a bad scene cannot exhaust the device.
  const int requested_cells = grid_cell_count(dimensions);
  const bool was_clamped = requested_cells > max_cells;
  const int cells = std::min(requested_cells, max_cells);
  if (grid.allocatedCells < cells || grid.allocatedParticles < n) {
    grid.alloc(std::max(cells, grid.allocatedCells), std::max(n, grid.allocatedParticles));
  }
  grid.min = minimum;
  grid.max = maximum;
  grid.cellSize = cell_size;
  grid.dims = dimensions;
  grid.numCells = cells;
  grid.clamped = was_clamped;
  // Warn once per grid; a clamped grid is a configuration problem, not a transient.
  if (was_clamped) {
    std::fprintf(stderr, "warning: uniform grid clamped from %d to %d cells\n",
                 requested_cells, cells);
  }
}

void launch_grid_build(const float4* pos, int n, UniformGrid& grid,
                       cudaStream_t stream) {
  if (n <= 0 || grid.numCells <= 0) {
    return;
  }
  clear_counts_kernel<<<(grid.numCells + 255) / 256, 256, 0, stream>>>(
      grid.cellCount, grid.numCells);
  CUDA_CHECK(cudaGetLastError());
  count_cells_kernel<<<(n + 255) / 256, 256, 0, stream>>>(pos, n, grid);
  CUDA_CHECK(cudaGetLastError());
  std::size_t required_bytes = 0;
  CUDA_CHECK(cub::DeviceScan::ExclusiveSum(
      grid.scanStorage, required_bytes, grid.cellCount, grid.cellStart,
      grid.numCells, stream));
  if (required_bytes > grid.scanStorageBytes) {
    cudaFree(grid.scanStorage);
    CUDA_CHECK(cudaMalloc(&grid.scanStorage, required_bytes));
    grid.scanStorageBytes = required_bytes;
  }
  CUDA_CHECK(cub::DeviceScan::ExclusiveSum(
      grid.scanStorage, grid.scanStorageBytes, grid.cellCount, grid.cellStart,
      grid.numCells, stream));
  CUDA_CHECK(cudaMemcpyAsync(grid.cellStart + grid.numCells, &n, sizeof(int),
                             cudaMemcpyHostToDevice, stream));
  CUDA_CHECK(cudaMemcpyAsync(grid.cellCursor, grid.cellStart,
                             sizeof(int) * grid.numCells, cudaMemcpyDeviceToDevice,
                             stream));
  place_cells_kernel<<<(n + 255) / 256, 256, 0, stream>>>(pos, n, grid);
  CUDA_CHECK(cudaGetLastError());
}
