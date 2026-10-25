#include "sim/particles.h"
#include "util/cuda_check.h"

#include <cuda_runtime.h>

#include <cstdio>
#include <cstdlib>

void launch_forces(ParticleSystem&, const SimParams&, cudaStream_t);
void launch_integrate(ParticleSystem&, const SimParams&, cudaStream_t);

int main(int argc, char** argv) {
  const int count = argc > 1 ? std::atoi(argv[1]) : 1000000;
  const int iterations = argc > 2 ? std::atoi(argv[2]) : 100;
  ParticleSystem particles;
  InitParams init;
  launch_particles_init(particles, count, 42, init, nullptr);
  SimParams params;
  cudaEvent_t start{}, stop{};
  CUDA_CHECK(cudaEventCreate(&start));
  CUDA_CHECK(cudaEventCreate(&stop));
  CUDA_CHECK(cudaDeviceSynchronize());
  CUDA_CHECK(cudaEventRecord(start));
  for (int i = 0; i < iterations; ++i) {
    launch_forces(particles, params, nullptr);
    launch_integrate(particles, params, nullptr);
  }
  CUDA_CHECK(cudaEventRecord(stop));
  CUDA_CHECK(cudaEventSynchronize(stop));
  float elapsed = 0.0f;
  CUDA_CHECK(cudaEventElapsedTime(&elapsed, start, stop));
  std::printf("particles=%d iterations=%d ms/step=%.6f\n", count, iterations,
              elapsed / iterations);
  CUDA_CHECK(cudaEventDestroy(start));
  CUDA_CHECK(cudaEventDestroy(stop));
  particles.free();
  return 0;
}
