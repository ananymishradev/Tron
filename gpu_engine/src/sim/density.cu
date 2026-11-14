#include "sim/density.h"

#include "util/cuda_check.h"

#include <algorithm>

namespace {

__global__ void particle_density_kernel(const float4* positions, int count,
                                        ViewPlane plane, DensityGrid grid,
                                        int stride) {
  const int index = (blockIdx.x * blockDim.x + threadIdx.x) * stride;
  if (index >= count) {
    return;
  }
  const float4 position = positions[index];
  // Both projections use x horizontally; only the vertical axis changes.
  const float a = plane == ViewPlane::TopXZ ? position.x : position.x;
  const float b = plane == ViewPlane::TopXZ ? position.z : position.y;
  const int x = static_cast<int>((a - grid.min_a) /
                                 (grid.max_a - grid.min_a) * grid.width);
  const int y = static_cast<int>((b - grid.min_b) /
                                 (grid.max_b - grid.min_b) * grid.height);
  // Reject out-of-view particles instead of clamping, or the edge bins would lie.
  if (x >= 0 && x < grid.width && y >= 0 && y < grid.height) {
    atomicAdd(&grid.particle_device[y * grid.width + x], 1u);
  }
}

__global__ void body_density_kernel(const float4* positions, int count,
                                    ViewPlane plane, DensityGrid grid) {
  const int index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= count) {
    return;
  }
  const float4 position = positions[index];
  const float a = position.x;
  const float b = plane == ViewPlane::TopXZ ? position.z : position.y;
  const int x = static_cast<int>((a - grid.min_a) /
                                 (grid.max_a - grid.min_a) * grid.width);
  const int y = static_cast<int>((b - grid.min_b) /
                                 (grid.max_b - grid.min_b) * grid.height);
  if (x >= 0 && x < grid.width && y >= 0 && y < grid.height) {
    grid.body_device[y * grid.width + x] = 1;
  }
}

}  // namespace

void launch_density(const ParticleSystem& particles, const RigidBodySoA& bodies,
                    ViewPlane plane, DensityGrid& grid, int sample_stride,
                    cudaStream_t stream) {
  if (grid.width <= 0 || grid.height <= 0) {
    grid.particles.clear();
    grid.bodies.clear();
    grid.max_count = 0;
    return;
  }
  const int cells = grid.width * grid.height;
  grid.particles.assign(cells, 0);
  grid.bodies.assign(cells, 0);
  grid.max_count = 0;
  grid.plane = plane;
  // Stride is a performance knob; 1 means every particle contributes.
  sample_stride = std::max(1, sample_stride);

  DensityGrid device_grid = grid;
  CUDA_CHECK(cudaMalloc(&device_grid.particle_device, sizeof(uint32_t) * cells));
  CUDA_CHECK(cudaMalloc(&device_grid.body_device, sizeof(uint8_t) * cells));
  CUDA_CHECK(cudaMemsetAsync(device_grid.particle_device, 0,
                             sizeof(uint32_t) * cells, stream));
  CUDA_CHECK(cudaMemsetAsync(device_grid.body_device, 0,
                             sizeof(uint8_t) * cells, stream));
  if (particles.n > 0) {
    const int work = (particles.n + sample_stride - 1) / sample_stride;
    particle_density_kernel<<<(work + 255) / 256, 256, 0, stream>>>(
        particles.posMass, particles.n, plane, device_grid, sample_stride);
    CUDA_CHECK(cudaGetLastError());
  }
  if (bodies.n > 0) {
    body_density_kernel<<<(bodies.n + 255) / 256, 256, 0, stream>>>(
        bodies.pos, bodies.n, plane, device_grid);
    CUDA_CHECK(cudaGetLastError());
  }
  CUDA_CHECK(cudaMemcpyAsync(grid.particles.data(), device_grid.particle_device,
                             sizeof(uint32_t) * cells, cudaMemcpyDeviceToHost,
                             stream));
  CUDA_CHECK(cudaMemcpyAsync(grid.bodies.data(), device_grid.body_device,
                             sizeof(uint8_t) * cells, cudaMemcpyDeviceToHost,
                             stream));
  CUDA_CHECK(cudaStreamSynchronize(stream));
  for (uint32_t count : grid.particles) {
    grid.max_count = std::max(grid.max_count, count);
  }
  CUDA_CHECK(cudaFree(device_grid.particle_device));
  CUDA_CHECK(cudaFree(device_grid.body_device));
}
