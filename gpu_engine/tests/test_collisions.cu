#include "test_util.h"
#include "util/cuda_check.h"
#include "sim/collisions.h"

#include <cuda_runtime.h>

#include <cmath>
#include <vector>

int main() {
  constexpr int count = 2;
  ParticleSystem particles;
  particles.alloc(count);
  const float4 positions[count] = {{-0.05f, 1.0f, 0.0f, 1.0f},
                                   {0.05f, 1.0f, 0.0f, 1.0f}};
  const float4 velocities[count] = {{1.0f, 0.0f, 0.0f, 0.1f},
                                    {-1.0f, 0.0f, 0.0f, 0.1f}};
  CUDA_CHECK(cudaMemcpy(particles.posMass, positions, sizeof(positions),
                        cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(particles.vel, velocities, sizeof(velocities),
                        cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(particles.posPrev, particles.posMass,
                        sizeof(positions), cudaMemcpyDeviceToDevice));
  CUDA_CHECK(cudaMemcpy(particles.velPrev, particles.vel, sizeof(velocities),
                        cudaMemcpyDeviceToDevice));
  UniformGrid grid;
  launch_grid_setup(grid, particles.posPrev, count, 0.5f, 128, nullptr);
  launch_grid_build(particles.posPrev, count, grid, nullptr);
  SimParams params;
  params.gravity = {0.0f, 0.0f, 0.0f};
  params.drag = 0.0f;
  ContactStats stats{};
  launch_collisions(particles, grid, params, &stats, nullptr);
  float4 output[count];
  CUDA_CHECK(cudaMemcpy(output, particles.vel, sizeof(output),
                        cudaMemcpyDeviceToHost));
  CHECK(output[0].x < 0.0f);
  CHECK(output[1].x > 0.0f);
  CHECK(stats.contacts == 1);

  ParticleSystem static_particle;
  static_particle.alloc(1);
  const float4 static_position{0.0f, 1.0f, 0.0f, 0.0f};
  CUDA_CHECK(cudaMemcpy(static_particle.posMass, &static_position,
                        sizeof(static_position), cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(static_particle.posPrev, static_particle.posMass,
                        sizeof(static_position), cudaMemcpyDeviceToDevice));
  launch_grid_setup(grid, static_particle.posPrev, 1, 0.5f, 128, nullptr);
  launch_grid_build(static_particle.posPrev, 1, grid, nullptr);
  launch_collisions(static_particle, grid, params, &stats, nullptr);
  CUDA_CHECK(cudaMemcpy(&output[0], static_particle.vel, sizeof(float4),
                        cudaMemcpyDeviceToHost));
  CHECK_NEAR(output[0].x, 0.0f, 1e-6f);

  grid.free();
  static_particle.free();
  particles.free();
  return 0;
}
