#pragma once

#include "sim/sim_params.h"

#include <vector>

struct RefParticle {
  Vec3 pos{};
  Vec3 vel{};
  float invMass = 0.0f;
};

inline void ref_step(std::vector<RefParticle>& particles, const SimParams& params,
                     int steps) {
  for (int step = 0; step < steps; ++step) {
    for (RefParticle& particle : particles) {
      const Vec3 acceleration =
          params.gravity - particle.vel * (params.drag * particle.invMass);
      particle.vel += acceleration * params.dt;
      particle.pos += particle.vel * params.dt;
    }
  }
}
