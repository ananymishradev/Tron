#pragma once

#include "sim/sim_params.h"

#include <cmath>
#include <vector>

struct RefCollisionParticle {
  Vec3 pos{};
  Vec3 vel{};
  float invMass = 0.0f;
  float radius = 0.0f;
};

inline void ref_collisions_step(std::vector<RefCollisionParticle>& particles,
                                const SimParams& params) {
  const std::vector<RefCollisionParticle> snapshot = particles;
  for (std::size_t i = 0; i < particles.size(); ++i) {
    Vec3 impulse{};
    Vec3 correction{};
    const auto& current = snapshot[i];
    for (std::size_t j = 0; j < particles.size(); ++j) {
      if (i == j || current.radius <= 0.0f || snapshot[j].radius <= 0.0f) {
        continue;
      }
      const Vec3 delta = current.pos - snapshot[j].pos;
      const float distance_squared = dot(delta, delta);
      const float radius_sum = current.radius + snapshot[j].radius;
      if (distance_squared <= 1e-12f ||
          distance_squared >= radius_sum * radius_sum) {
        continue;
      }
      const Vec3 normal = normalize(delta);
      const Vec3 relative_velocity = current.vel - snapshot[j].vel;
      const float normal_velocity = dot(relative_velocity, normal);
      const float inverse_mass_sum = current.invMass + snapshot[j].invMass;
      if (inverse_mass_sum > 0.0f && normal_velocity < 0.0f) {
        impulse += normal * (-(1.0f + params.restitution) * normal_velocity /
                             inverse_mass_sum);
      }
      if (inverse_mass_sum > 0.0f) {
        correction += normal * (0.8f * (radius_sum - sqrt(distance_squared)) *
                                current.invMass / inverse_mass_sum);
      }
    }
    particles[i].pos = current.pos + correction;
    particles[i].vel = current.vel + impulse;
    if (particles[i].pos.y - current.radius < params.planeY &&
        current.radius > 0.0f) {
      particles[i].pos.y = params.planeY + current.radius;
      if (particles[i].vel.y < 0.0f) {
        particles[i].vel.y = -params.restitution * particles[i].vel.y;
      }
    }
  }
}
