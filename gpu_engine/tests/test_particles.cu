#include "test_util.h"
#include "util/cuda_check.h"
#include "sim/particles.h"

#include <cuda_runtime.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <vector>

int main() {
  constexpr int count = 1000;
  const InitParams params{};
  ParticleSystem first;
  ParticleSystem second;
  launch_particles_init(first, count, 42, params, nullptr);
  launch_particles_init(second, count, 42, params, nullptr);
  CUDA_CHECK(cudaDeviceSynchronize());

  std::vector<float4> positions(count);
  std::vector<float4> velocities(count);
  std::vector<float4> second_positions(count);
  std::vector<int> materials(count);
  CUDA_CHECK(cudaMemcpy(positions.data(), first.posMass, sizeof(float4) * count,
                        cudaMemcpyDeviceToHost));
  CUDA_CHECK(cudaMemcpy(velocities.data(), first.vel, sizeof(float4) * count,
                        cudaMemcpyDeviceToHost));
  CUDA_CHECK(cudaMemcpy(second_positions.data(), second.posMass,
                        sizeof(float4) * count, cudaMemcpyDeviceToHost));
  CUDA_CHECK(cudaMemcpy(materials.data(), first.material, sizeof(int) * count,
                        cudaMemcpyDeviceToHost));

  for (int i = 0; i < count; ++i) {
    CHECK(positions[i].x >= params.boxMin.x && positions[i].x <= params.boxMax.x);
    CHECK(positions[i].y >= params.boxMin.y && positions[i].y <= params.boxMax.y);
    CHECK(positions[i].z >= params.boxMin.z && positions[i].z <= params.boxMax.z);
    const float mass = 1.0f / positions[i].w;
    CHECK(mass >= params.massMin && mass <= params.massMax);
    CHECK_NEAR(velocities[i].w, params.radius, 1e-7f);
    CHECK(materials[i] == params.material);
  }
  CHECK(std::memcmp(positions.data(), second_positions.data(),
                    sizeof(float4) * count) == 0);

  first.free();
  second.free();
  return 0;
}
