#include "sim/collisions.h"
#include "sim/grid.h"
#include "sim/particles.h"
#include "util/cuda_check.h"

#include <cuda_runtime.h>

#include <cstdio>
#include <cstdlib>

int main(int argc, char** argv) {
  const int count = argc > 1 ? std::atoi(argv[1]) : 1000000;
  const int iterations = argc > 2 ? std::atoi(argv[2]) : 100;
  ParticleSystem particles;
  launch_particles_init(particles, count, 42, InitParams{}, nullptr);
  UniformGrid grid;
  SimParams params;
  ContactStats stats{};
  cudaEvent_t start{}, stop{};
  CUDA_CHECK(cudaEventCreate(&start));
  CUDA_CHECK(cudaEventCreate(&stop));
  CUDA_CHECK(cudaDeviceSynchronize());
  launch_grid_setup(grid, particles.posMass, count, 0.25f, 1 << 20, nullptr);
  CUDA_CHECK(cudaEventRecord(start));
  for (int i = 0; i < iterations; ++i) {
    CUDA_CHECK(cudaMemcpy(particles.posPrev, particles.posMass,
                          sizeof(float4) * count, cudaMemcpyDeviceToDevice));
    CUDA_CHECK(cudaMemcpy(particles.velPrev, particles.vel,
                          sizeof(float4) * count, cudaMemcpyDeviceToDevice));
    launch_grid_build(particles.posPrev, count, grid, nullptr);
    launch_collisions(particles, grid, params, &stats, nullptr);
  }
  CUDA_CHECK(cudaEventRecord(stop));
  CUDA_CHECK(cudaEventSynchronize(stop));
  float elapsed = 0.0f;
  CUDA_CHECK(cudaEventElapsedTime(&elapsed, start, stop));
  std::printf("particles=%d iterations=%d ms/step=%.6f pairs=%u contacts=%u\n",
              count, iterations, elapsed / iterations, stats.pairTests,
              stats.contacts);
  grid.free();
  particles.free();
  CUDA_CHECK(cudaEventDestroy(start));
  CUDA_CHECK(cudaEventDestroy(stop));
  return 0;
}
