#include "sim/grid.h"
#include "sim/particles.h"
#include "util/cuda_check.h"

#include <cuda_runtime.h>

#include <cstdio>
#include <cstdlib>

int main(int argc, char** argv) {
  const int count = argc > 1 ? std::atoi(argv[1]) : 1000000;
  ParticleSystem particles;
  launch_particles_init(particles, count, 42, InitParams{}, nullptr);
  UniformGrid grid;
  cudaEvent_t start{}, stop{};
  CUDA_CHECK(cudaEventCreate(&start));
  CUDA_CHECK(cudaEventCreate(&stop));
  CUDA_CHECK(cudaDeviceSynchronize());
  CUDA_CHECK(cudaEventRecord(start));
  launch_grid_setup(grid, particles.posMass, count, 0.5f, 1 << 20, nullptr);
  launch_grid_build(particles.posMass, count, grid, nullptr);
  CUDA_CHECK(cudaEventRecord(stop));
  CUDA_CHECK(cudaEventSynchronize(stop));
  float elapsed = 0.0f;
  CUDA_CHECK(cudaEventElapsedTime(&elapsed, start, stop));
  std::printf("particles=%d grid_ms=%.6f cells=%d clamped=%s\n", count, elapsed,
              grid.numCells, grid.clamped ? "yes" : "no");
  grid.free();
  particles.free();
  CUDA_CHECK(cudaEventDestroy(start));
  CUDA_CHECK(cudaEventDestroy(stop));
  return 0;
}
