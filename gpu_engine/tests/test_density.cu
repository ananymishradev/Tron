#include "sim/density.h"
#include "sim/rigid.h"
#include "sim/simulation.h"
#include "test_util.h"
#include "util/cuda_check.h"

int main() {
  ParticleSystem particles;
  particles.alloc(4);
  const float4 positions[4] = {
      {-1.0f, 0.0f, -1.0f, 1.0f}, {1.0f, 0.0f, 1.0f, 1.0f},
      {1.0f, 0.0f, -1.0f, 1.0f}, {-1.0f, 0.0f, 1.0f, 1.0f}};
  CUDA_CHECK(cudaMemcpy(particles.posMass, positions, sizeof(positions),
                        cudaMemcpyHostToDevice));
  DensityGrid grid;
  grid.width = 4;
  grid.height = 4;
  grid.min_a = -2.0f;
  grid.max_a = 2.0f;
  grid.min_b = -2.0f;
  grid.max_b = 2.0f;
  launch_density(particles, RigidBodySoA{}, ViewPlane::TopXZ, grid, 1, nullptr);
  CHECK(grid.max_count == 1);
  CHECK(grid.particles.size() == 16);
  CHECK(grid.particles[0] == 0);
  CHECK(grid.particles[5] == 1);
  CHECK(grid.particles[15] == 1);
  particles.free();
  return 0;
}
