#include "sim/particles.h"

#include "util/cuda_check.h"

__global__ void integrate_kernel(float4* pos_mass, float4* velocity,
                                 const float4* force, int count,
                                 SimParams params) {
  const int index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= count) {
    return;
  }
  const float inv_mass = pos_mass[index].w;
  if (inv_mass <= 0.0f) {
    return;
  }
  velocity[index].x += params.dt * force[index].x * inv_mass;
  velocity[index].y += params.dt * force[index].y * inv_mass;
  velocity[index].z += params.dt * force[index].z * inv_mass;
  pos_mass[index].x += params.dt * velocity[index].x;
  pos_mass[index].y += params.dt * velocity[index].y;
  pos_mass[index].z += params.dt * velocity[index].z;
}

void launch_integrate(ParticleSystem& particles, const SimParams& params,
                      cudaStream_t stream) {
  if (particles.n <= 0) {
    return;
  }
  integrate_kernel<<<(particles.n + 255) / 256, 256, 0, stream>>>(
      particles.posMass, particles.vel, particles.force, particles.n, params);
  CUDA_CHECK(cudaGetLastError());
}
