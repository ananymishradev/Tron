#pragma once

#include "util/math.h"

#include <cuda_runtime.h>

// Uniform grid in cell-list form. sortedIdx holds particle indices, not reordered particles.
struct UniformGrid {
  Vec3 min{};
  Vec3 max{};
  float cellSize = 1.0f;
  int3 dims{1, 1, 1};
  int numCells = 0;
  bool clamped = false;
  bool clamp_warning_emitted = false;
  int* cellCount = nullptr;
  int* cellStart = nullptr;
  int* cellCursor = nullptr;
  // Counting-sort output: cellStart[cell]..cellStart[cell+1] indexes into this array.
  int* sortedIdx = nullptr;
  int allocatedCells = 0;
  int allocatedParticles = 0;
  void* scanStorage = nullptr;
  std::size_t scanStorageBytes = 0;

  void alloc(int maxCells, int n);
  void free();
};

int grid_cell_count(int3 dims);
bool grid_clamped(const UniformGrid& grid);
int grid_linear_cell(float3 position, const UniformGrid& grid);
void launch_grid_setup(UniformGrid& grid, const float4* pos, int n,
                       float cellSize, int maxCells, cudaStream_t stream);
void launch_grid_build(const float4* pos, int n, UniformGrid& grid,
                       cudaStream_t stream);
