#pragma once

#include "sim/simulation.h"

void launch_density(const ParticleSystem& particles, const RigidBodySoA& bodies,
                    ViewPlane plane, DensityGrid& grid, int sample_stride,
                    cudaStream_t stream);
