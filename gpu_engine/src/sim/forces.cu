#include "sim/particles.h"

#include "util/cuda_check.h"

__global__ void forces_kernel(float4* force, const float4* pos_mass,
                              const float4* velocity, int count,
                              SimParams params) {
  const int index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= count) {
    return;
  }
  const float inv_mass = pos_mass[index].w;
  // Static or infinite-mass particles get no force; clear the buffer and return.
  if (inv_mass <= 0.0f) {
    force[index] = {0.0f, 0.0f, 0.0f, 0.0f};
    return;
  }
  const float mass = 1.0f / inv_mass;
  force[index] = {mass * params.gravity.x - params.drag * velocity[index].x,
                  mass * params.gravity.y - params.drag * velocity[index].y,
                  mass * params.gravity.z - params.drag * velocity[index].z, 0.0f};
}

void launch_forces(ParticleSystem& particles, const SimParams& params,
                   cudaStream_t stream) {
  if (particles.n <= 0) {
    return;
  }
  forces_kernel<<<(particles.n + 255) / 256, 256, 0, stream>>>(
      particles.force, particles.posMass, particles.vel, particles.n, params);
  CUDA_CHECK(cudaGetLastError());
}
