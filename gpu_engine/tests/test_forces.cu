#include "test_util.h"
#include "util/cuda_check.h"
#include "sim/particles.h"

#include <cuda_runtime.h>

#include <cmath>
#include <vector>

void launch_forces(ParticleSystem&, const SimParams&, cudaStream_t);
void launch_integrate(ParticleSystem&, const SimParams&, cudaStream_t);
void launch_stats(const ParticleSystem&, const SimParams&, SimStats*, cudaStream_t);

static void step(ParticleSystem& particles, const SimParams& params, int count) {
  for (int i = 0; i < count; ++i) {
    launch_forces(particles, params, nullptr);
    launch_integrate(particles, params, nullptr);
  }
  CUDA_CHECK(cudaDeviceSynchronize());
}

int main() {
  ParticleSystem particles;
  particles.alloc(1);
  float4 position{0.0f, 10.0f, 0.0f, 1.0f};
  float4 velocity{0.0f, 0.0f, 0.0f, 0.1f};
  CUDA_CHECK(cudaMemcpy(particles.posMass, &position, sizeof(position),
                        cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(particles.vel, &velocity, sizeof(velocity),
                        cudaMemcpyHostToDevice));
  SimParams gravity_only;
  gravity_only.drag = 0.0f;
  step(particles, gravity_only, 60);
  CUDA_CHECK(cudaMemcpy(&position, particles.posMass, sizeof(position),
                        cudaMemcpyDeviceToHost));
  CUDA_CHECK(cudaMemcpy(&velocity, particles.vel, sizeof(velocity),
                        cudaMemcpyDeviceToHost));
  CHECK_NEAR(velocity.y, gravity_only.gravity.y, 1e-5f);
  CHECK_NEAR(position.y, 10.0f + gravity_only.gravity.y * 0.5f, 0.1f);
  particles.free();

  particles.alloc(1);
  position = {0.0f, 0.0f, 0.0f, 1.0f};
  velocity = {1.0f, 0.0f, 0.0f, 0.1f};
  CUDA_CHECK(cudaMemcpy(particles.posMass, &position, sizeof(position),
                        cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(particles.vel, &velocity, sizeof(velocity),
                        cudaMemcpyHostToDevice));
  SimParams drag_only;
  drag_only.gravity = {0.0f, 0.0f, 0.0f};
  drag_only.drag = 0.1f;
  step(particles, drag_only, 60);
  CUDA_CHECK(cudaMemcpy(&velocity, particles.vel, sizeof(velocity),
                        cudaMemcpyDeviceToHost));
  CHECK_NEAR(velocity.x, std::exp(-0.1f), 2e-2f);
  particles.free();

  particles.alloc(2);
  const float4 positions[2] = {{0.0f, 1.0f, 0.0f, 1.0f},
                               {0.0f, 1.0f, 0.0f, 1.0f}};
  const float4 velocities[2] = {{1.0f, 0.0f, 0.0f, 0.1f},
                                {-1.0f, 0.0f, 0.0f, 0.1f}};
  CUDA_CHECK(cudaMemcpy(particles.posMass, positions, sizeof(positions),
                        cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(particles.vel, velocities, sizeof(velocities),
                        cudaMemcpyHostToDevice));
  SimParams no_force;
  no_force.gravity = {0.0f, 0.0f, 0.0f};
  no_force.drag = 0.0f;
  SimStats* device_stats = nullptr;
  CUDA_CHECK(cudaMalloc(&device_stats, sizeof(SimStats)));
  launch_stats(particles, no_force, device_stats, nullptr);
  SimStats stats{};
  CUDA_CHECK(cudaMemcpy(&stats, device_stats, sizeof(stats),
                        cudaMemcpyDeviceToHost));
  CHECK_NEAR(stats.momentum[0], 0.0f, 1e-6f);
  CHECK(stats.firstNonFinite == -1);

  float4 bad_position = positions[0];
  bad_position.y = NAN;
  CUDA_CHECK(cudaMemcpy(particles.posMass, &bad_position, sizeof(bad_position),
                        cudaMemcpyHostToDevice));
  launch_stats(particles, no_force, device_stats, nullptr);
  CUDA_CHECK(cudaMemcpy(&stats, device_stats, sizeof(stats),
                        cudaMemcpyDeviceToHost));
  CHECK(stats.firstNonFinite == 0);
  CUDA_CHECK(cudaFree(device_stats));
  particles.free();
  return 0;
}
