#include "sim/collisions.h"
#include "sim/grid.h"
#include "sim/particles.h"
#include "util/cuda_check.h"

#include <cuda_runtime.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <vector>

void launch_forces(ParticleSystem&, const SimParams&, cudaStream_t);
void launch_integrate(ParticleSystem&, const SimParams&, cudaStream_t);

struct StageTimes {
  float grid = 0.0f;
  float forces = 0.0f;
  float integrate = 0.0f;
  float collisions = 0.0f;
};

static float elapsed(cudaEvent_t start, cudaEvent_t stop) {
  float milliseconds = 0.0f;
  CUDA_CHECK(cudaEventElapsedTime(&milliseconds, start, stop));
  return milliseconds;
}

int main(int argc, char** argv) {
  const int count = argc > 1 ? std::atoi(argv[1]) : 1000000;
  const int warmup = 100;
  const int measured = argc > 3 ? std::atoi(argv[3]) : 300;
  ParticleSystem particles;
  particles.alloc(count);
  launch_particles_init(particles, count, 42, InitParams{}, nullptr);
  UniformGrid grid;
  SimParams params;
  ContactStats contacts{};
  cudaEvent_t start{}, stop{};
  CUDA_CHECK(cudaEventCreate(&start));
  CUDA_CHECK(cudaEventCreate(&stop));
  std::vector<StageTimes> samples;
  samples.reserve(measured);
  for (int frame = 0; frame < warmup + measured; ++frame) {
    StageTimes sample{};
    CUDA_CHECK(cudaMemcpy(particles.posPrev, particles.posMass,
                          sizeof(float4) * count, cudaMemcpyDeviceToDevice));
    CUDA_CHECK(cudaMemcpy(particles.velPrev, particles.vel,
                          sizeof(float4) * count, cudaMemcpyDeviceToDevice));
    CUDA_CHECK(cudaEventRecord(start));
    launch_grid_setup(grid, particles.posPrev, count, 0.25f, 1 << 20, nullptr);
    launch_grid_build(particles.posPrev, count, grid, nullptr);
    CUDA_CHECK(cudaEventRecord(stop));
    CUDA_CHECK(cudaEventSynchronize(stop));
    sample.grid = elapsed(start, stop);

    CUDA_CHECK(cudaEventRecord(start));
    launch_forces(particles, params, nullptr);
    CUDA_CHECK(cudaEventRecord(stop));
    CUDA_CHECK(cudaEventSynchronize(stop));
    sample.forces = elapsed(start, stop);

    CUDA_CHECK(cudaEventRecord(start));
    launch_integrate(particles, params, nullptr);
    CUDA_CHECK(cudaEventRecord(stop));
    CUDA_CHECK(cudaEventSynchronize(stop));
    sample.integrate = elapsed(start, stop);

    CUDA_CHECK(cudaEventRecord(start));
    launch_collisions(particles, grid, params, &contacts, nullptr);
    CUDA_CHECK(cudaEventRecord(stop));
    CUDA_CHECK(cudaEventSynchronize(stop));
    sample.collisions = elapsed(start, stop);
    if (frame >= warmup) {
      samples.push_back(sample);
    }
  }
  StageTimes mean{};
  for (const StageTimes& sample : samples) {
    mean.grid += sample.grid;
    mean.forces += sample.forces;
    mean.integrate += sample.integrate;
    mean.collisions += sample.collisions;
  }
  const float divisor = static_cast<float>(samples.size());
  std::printf("particles=%d frames=%d grid_ms=%.3f forces_ms=%.3f "
              "integrate_ms=%.3f collisions_ms=%.3f\n",
              count, measured, mean.grid / divisor, mean.forces / divisor,
              mean.integrate / divisor, mean.collisions / divisor);
  CUDA_CHECK(cudaEventDestroy(start));
  CUDA_CHECK(cudaEventDestroy(stop));
  grid.free();
  particles.free();
  return 0;
}
