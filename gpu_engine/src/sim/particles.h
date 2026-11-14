#pragma once

#include "sim/sim_params.h"

#include <cuda_runtime.h>

#include <cstdint>

// SoA layout: one float4 per particle keeps the hot arrays coalesced.
struct ParticleSystem {
  int n = 0;
  float4* posMass = nullptr;
  float4* vel = nullptr;
  // posPrev/velPrev are the broadphase snapshot; collisions read these while integration writes the live state.
  float4* posPrev = nullptr;
  float4* velPrev = nullptr;
  float4* force = nullptr;
  int* material = nullptr;

  void alloc(int count);
  void free();
};

void launch_particles_init(ParticleSystem& particles, int n, uint32_t seed,
                           const InitParams& params, cudaStream_t stream);
void launch_forces(ParticleSystem&, const SimParams&, cudaStream_t);
void launch_integrate(ParticleSystem&, const SimParams&, cudaStream_t);
void launch_stats(const ParticleSystem&, const SimParams&, SimStats*, cudaStream_t);
