#include "sim/particles.h"

#include "util/cuda_check.h"

#include <cuda_runtime.h>

#include <cstdint>

namespace {

// SplitMix64 gives a stable, seedable stream without keeping RNG state around.
__device__ uint64_t splitmix64(uint64_t value) {
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31);
}

__device__ float unit_random(uint64_t seed, uint64_t index, uint64_t stream) {
  const uint64_t bits = splitmix64(seed ^ (index * 0x9e3779b97f4a7c15ULL) ^
                                    (stream * 0xd1b54a32d192ed03ULL));
  return static_cast<float>((bits >> 40) & 0xFFFFFF) / 16777216.0f;
}

__global__ void init_kernel(float4* pos_mass, float4* velocity, int* material,
                            int count, uint32_t seed, InitParams params) {
  const int index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= count) {
    return;
  }

  const float rx = unit_random(seed, index, 0);
  const float ry = unit_random(seed, index, 1);
  const float rz = unit_random(seed, index, 2);
  const float mass_random = unit_random(seed, index, 3);
  const float speed_random = unit_random(seed, index, 4);
  const float mass = params.massMin + mass_random * (params.massMax - params.massMin);

  // Store inverse mass in w; force integration can multiply instead of divide.
  pos_mass[index] = {
      params.boxMin.x + rx * (params.boxMax.x - params.boxMin.x),
      params.boxMin.y + ry * (params.boxMax.y - params.boxMin.y),
      params.boxMin.z + rz * (params.boxMax.z - params.boxMin.z),
      1.0f / mass};
  velocity[index] = {
      (speed_random - 0.5f) * 0.2f,
      (unit_random(seed, index, 5) - 0.5f) * 0.2f,
      (unit_random(seed, index, 6) - 0.5f) * 0.2f,
      params.radius};
  material[index] = params.material;
}

}  // namespace

void ParticleSystem::alloc(int count) {
  free();
  n = count;
  if (count <= 0) {
    return;
  }
  CUDA_CHECK(cudaMalloc(&posMass, sizeof(float4) * count));
  CUDA_CHECK(cudaMalloc(&vel, sizeof(float4) * count));
  CUDA_CHECK(cudaMalloc(&posPrev, sizeof(float4) * count));
  CUDA_CHECK(cudaMalloc(&velPrev, sizeof(float4) * count));
  CUDA_CHECK(cudaMalloc(&force, sizeof(float4) * count));
  CUDA_CHECK(cudaMalloc(&material, sizeof(int) * count));
  CUDA_CHECK(cudaMemset(vel, 0, sizeof(float4) * count));
  CUDA_CHECK(cudaMemset(velPrev, 0, sizeof(float4) * count));
  CUDA_CHECK(cudaMemset(force, 0, sizeof(float4) * count));
}

void ParticleSystem::free() {
  cudaFree(posMass);
  cudaFree(vel);
  cudaFree(posPrev);
  cudaFree(velPrev);
  cudaFree(force);
  cudaFree(material);
  n = 0;
  posMass = nullptr;
  vel = nullptr;
  posPrev = nullptr;
  velPrev = nullptr;
  force = nullptr;
  material = nullptr;
}

void launch_particles_init(ParticleSystem& particles, int n, uint32_t seed,
                           const InitParams& params, cudaStream_t stream) {
  if (particles.n != n) {
    particles.alloc(n);
  }
  if (n <= 0) {
    return;
  }
  init_kernel<<<(n + 255) / 256, 256, 0, stream>>>(
      particles.posMass, particles.vel, particles.material, n, seed, params);
  CUDA_CHECK(cudaGetLastError());
}
