#include "sim/particles.h"

#include "util/cuda_check.h"

#include <climits>

__global__ void stats_kernel(const float4* pos_mass, const float4* velocity,
                             int count, SimParams params, SimStats* output) {
  const int index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= count) {
    return;
  }
  const float4 position = pos_mass[index];
  const float4 speed = velocity[index];
  const float inv_mass = position.w;
  if (!isfinite(position.x) || !isfinite(position.y) || !isfinite(position.z) ||
      !isfinite(speed.x) || !isfinite(speed.y) || !isfinite(speed.z) ||
      !isfinite(inv_mass)) {
    // Keep the lowest bad index so the UI can point at a concrete particle.
    atomicMin(&output->firstNonFinite, index);
    return;
  }
  if (inv_mass <= 0.0f) {
    return;
  }
  const float mass = 1.0f / inv_mass;
  atomicAdd(&output->kinetic,
            0.5f * mass * (speed.x * speed.x + speed.y * speed.y +
                           speed.z * speed.z));
  atomicAdd(&output->potential,
            -mass * (params.gravity.x * position.x +
                     params.gravity.y * position.y + params.gravity.z * position.z));
  atomicAdd(&output->momentum[0], mass * speed.x);
  atomicAdd(&output->momentum[1], mass * speed.y);
  atomicAdd(&output->momentum[2], mass * speed.z);
}

__global__ void stats_init_kernel(SimStats* output) {
  if (blockIdx.x == 0 && threadIdx.x == 0) {
    output->kinetic = 0.0f;
    output->potential = 0.0f;
    output->momentum[0] = 0.0f;
    output->momentum[1] = 0.0f;
    output->momentum[2] = 0.0f;
    output->firstNonFinite = INT_MAX;
  }
}

// Turn the sentinel back into -1 so callers can use a simple >= 0 check.
__global__ void stats_finalize_kernel(SimStats* output) {
  if (blockIdx.x == 0 && threadIdx.x == 0 &&
      output->firstNonFinite == INT_MAX) {
    output->firstNonFinite = -1;
  }
}

void launch_stats(const ParticleSystem& particles, const SimParams& params,
                  SimStats* output, cudaStream_t stream) {
  stats_init_kernel<<<1, 1, 0, stream>>>(output);
  CUDA_CHECK(cudaGetLastError());
  if (particles.n > 0) {
    stats_kernel<<<(particles.n + 255) / 256, 256, 0, stream>>>(
        particles.posMass, particles.vel, particles.n, params, output);
    CUDA_CHECK(cudaGetLastError());
  }
  stats_finalize_kernel<<<1, 1, 0, stream>>>(output);
  CUDA_CHECK(cudaGetLastError());
}
