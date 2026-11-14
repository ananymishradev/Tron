#include "sim/grid.h"

#include "util/cuda_check.h"

#include <cub/cub.cuh>
#include <cuda_runtime.h>

#include <algorithm>
#include <cfloat>
#include <climits>
#include <cmath>
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

__global__ void reduce_aabb_kernel(const float4* positions, int count,
                                   float* output) {
    __shared__ float minimum[3][256];
    __shared__ float maximum[3][256];
    const int lane = threadIdx.x;
    float min_x = FLT_MAX;
    float min_y = FLT_MAX;
    float min_z = FLT_MAX;
    float max_x = -FLT_MAX;
    float max_y = -FLT_MAX;
    float max_z = -FLT_MAX;
    for (int index = lane; index < count; index += blockDim.x) {
      const float4 value = positions[index];
      if (!isfinite(value.x) || !isfinite(value.y) || !isfinite(value.z)) {
        continue;
      }
      min_x = fminf(min_x, value.x);
      min_y = fminf(min_y, value.y);
      min_z = fminf(min_z, value.z);
      max_x = fmaxf(max_x, value.x);
      max_y = fmaxf(max_y, value.y);
      max_z = fmaxf(max_z, value.z);
    }
    minimum[0][lane] = min_x;
    minimum[1][lane] = min_y;
    minimum[2][lane] = min_z;
    maximum[0][lane] = max_x;
    maximum[1][lane] = max_y;
    maximum[2][lane] = max_z;
    __syncthreads();
    for (int stride = blockDim.x / 2; stride > 0; stride /= 2) {
      if (lane < stride) {
        for (int axis = 0; axis < 3; ++axis) {
          minimum[axis][lane] = fminf(minimum[axis][lane], minimum[axis][lane + stride]);
          maximum[axis][lane] = fmaxf(maximum[axis][lane], maximum[axis][lane + stride]);
      }
      __syncthreads();
    }
    if (lane == 0) {
      for (int axis = 0; axis < 3; ++axis) {
        output[axis] = minimum[axis][0];
        output[axis + 3] = maximum[axis][0];
      }
    }
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
  CUDA_CHECK(cudaMalloc(&aabbScratch, sizeof(float) * 6));
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
  cudaFree(aabbScratch);
  cellCount = nullptr;
  cellStart = nullptr;
  cellCursor = nullptr;
  sortedIdx = nullptr;
  scanStorage = nullptr;
  scanStorageBytes = 0;
  aabbScratch = nullptr;
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
  if (grid.aabbScratch == nullptr) {
    CUDA_CHECK(cudaMalloc(&grid.aabbScratch, sizeof(float) * 6));
  }

  const float initial[6] = {FLT_MAX, FLT_MAX, FLT_MAX,
                            -FLT_MAX, -FLT_MAX, -FLT_MAX};
  CUDA_CHECK(cudaMemcpyAsync(grid.aabbScratch, initial, sizeof(initial),
                             cudaMemcpyHostToDevice, stream));
  reduce_aabb_kernel<<<1, 256, 0, stream>>>(pos, n, grid.aabbScratch);
  CUDA_CHECK(cudaGetLastError());
  float bounds[6]{};
  CUDA_CHECK(cudaMemcpyAsync(bounds, grid.aabbScratch, sizeof(bounds),
                             cudaMemcpyDeviceToHost, stream));
  CUDA_CHECK(cudaStreamSynchronize(stream));
  Vec3 minimum{bounds[0], bounds[1], bounds[2]};
  Vec3 maximum{bounds[3], bounds[4], bounds[5]};
  if (!std::isfinite(minimum.x) || !std::isfinite(minimum.y) ||
      !std::isfinite(minimum.z)) {
    minimum = {0.0f, 0.0f, 0.0f};
    maximum = {0.0f, 0.0f, 0.0f};
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
    if (!grid.clamp_warning_emitted) {
      std::fprintf(stderr, "warning: uniform grid clamped from %d to %d cells\n",
                   requested_cells, cells);
      grid.clamp_warning_emitted = true;
    }
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
  CUDA_CHECK(cudaStreamSynchronize(stream));
  std::size_t required_bytes = 0;
  CUDA_CHECK(cub::DeviceScan::ExclusiveSum(
      // Pass a null buffer to ask CUB for the scratch size.
      nullptr, required_bytes, grid.cellCount, grid.cellStart,
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
