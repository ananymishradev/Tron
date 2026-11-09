#pragma once

#include "sim/grid.h"
#include "sim/particles.h"

// Counters are halved on the host because each pair is visited from both sides.
struct ContactStats {
  unsigned int pairTests = 0;
  unsigned int contacts = 0;
};

void launch_collisions(ParticleSystem& particles, const UniformGrid& grid,
                       const SimParams& params, ContactStats* stats,
                       cudaStream_t stream);
// Benchmark entry point; this checkout maps it to the scalar kernel.
void launch_collisions_ilp(ParticleSystem& particles, const UniformGrid& grid,
                           const SimParams& params, ContactStats* stats,
                           cudaStream_t stream);
